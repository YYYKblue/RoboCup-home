#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf2_ros
import math
import tf_conversions
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_srvs.srv import Empty  # 用于清理代价地图
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class ProgressiveExplorer:
    """
    渐进式探索器模块
    边进行扇形探索，边寻找 1.5m 内的可停靠点。一旦发现即直接导航，且面向顾客。
    """
    def __init__(self):
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = None
        self.failed_waypoints = set() # 黑名单集合
        
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.loginfo("[Explorer] 正在等待获取地图...")
        while self.map_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 连接服务
        self.make_plan_service = "/move_base/make_plan"
        self.clear_costmaps_service = "/move_base/clear_costmaps"
        
        rospy.loginfo("[Explorer] 等待导航服务连接...")
        try:
            rospy.wait_for_service(self.make_plan_service, timeout=10.0)
            self.make_plan_client = rospy.ServiceProxy(self.make_plan_service, GetPlan)
            rospy.wait_for_service(self.clear_costmaps_service, timeout=5.0)
            self.clear_costmaps_client = rospy.ServiceProxy(self.clear_costmaps_service, Empty)
            rospy.loginfo("[Explorer] 规划与清理服务连接成功！")
        except rospy.ROSException as e:
            rospy.logerr(f"[Explorer] 服务连接失败: {e}")

        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("[Explorer] 节点初始化完成！")

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_width = msg.info.width

    def get_map_value(self, x, y):
        """0:空闲, 100:障碍, -1:未知"""
        if self.map_origin is None: return -1
        ix = int((x - self.map_origin.position.x) / self.map_resolution)
        iy = int((y - self.map_origin.position.y) / self.map_resolution)
        index = iy * self.map_width + ix
        if 0 <= index < len(self.map_data):
            return self.map_data[index]
        return -1 

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            return trans.transform.translation.x, trans.transform.translation.y
        except Exception:
            return None, None

    def get_robot_pose_stamped(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception:
            return None

    def is_reachable(self, start_pose, target_x, target_y):
        if start_pose is None: return False
        req = GetPlanRequest()
        req.start = start_pose
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = target_x
        req.goal.pose.position.y = target_y
        req.goal.pose.orientation.w = 1.0 
        req.tolerance = 0.05
        try:
            res = self.make_plan_client(req)
            return bool(res.plan.poses and len(res.plan.poses) > 0)
        except rospy.ServiceException:
            return False

    def clear_costmaps(self):
        try:
            self.clear_costmaps_client()
            rospy.loginfo("[Explorer] 代价地图已清理，解除潜在卡死。")
        except rospy.ServiceException as e:
            rospy.logwarn(f"[Explorer] 清理代价地图失败: {e}")

    def generate_spiral_candidates(self, rx, ry, target_x, target_y):
        """全向扇形搜索中转点，作为防死胡同的推进方式"""
        candidates =[]
        vec_x = target_x - rx
        vec_y = target_y - ry
        base_dist = math.hypot(vec_x, vec_y)
        base_angle = math.atan2(vec_y, vec_x)
        
        angle_offsets = [0.0]
        for deg in range(15, 181, 15):
            rad = math.radians(deg)
            angle_offsets.append(rad)
            if deg != 180:
                angle_offsets.append(-rad)
        
        for offset in angle_offsets:
            search_angle = base_angle + offset
            dir_x = math.cos(search_angle)
            dir_y = math.sin(search_angle)
            
            step_size = self.map_resolution
            current_dist = 0.25 
            max_search_dist = base_dist + 1.5 
            frontier_dist = max_search_dist
            
            while current_dist < max_search_dist:
                check_x = rx + dir_x * current_dist
                check_y = ry + dir_y * current_dist
                val = self.get_map_value(check_x, check_y)
                if val == -1 or val > 50: 
                    frontier_dist = current_dist
                    break
                current_dist += step_size
                
            test_dist = frontier_dist - 0.2 
            
            while test_dist > 0.25:
                wp_x = rx + dir_x * test_dist
                wp_y = ry + dir_y * test_dist
                
                if self.get_map_value(wp_x, wp_y) == 0:
                     dist_to_target = math.hypot(target_x - wp_x, target_y - wp_y)
                     cost = dist_to_target + abs(offset) * 0.15 
                     candidates.append( (cost, wp_x, wp_y, search_angle) )
                
                test_dist -= 0.15
                
        candidates.sort(key=lambda x: x[0])
        return candidates

    def check_1_5m_stop_point(self, rx, ry, target_x, target_y, robot_pose_stamped):
        """在目标点周围寻找可以停靠的点，半径控制在 0.7m 到 1.5m 之间。
        如果找到了安全、可达的点，计算朝向面向目标的角度并返回。
        """
        candidates =[]
        
        # 机器人到目标的方向，作为寻找停靠点的基准面 (优先在面对机器人的这一侧找)
        base_angle = math.atan2(ry - target_y, rx - target_x)
        
        # 定义搜索半径：0.7米到1.5米（不低于0.7米防止撞到顾客）
        radii =[1.1,0.9,1.3,0.7,1.5]
        
        # 扇形角度展开，±0到±90度，覆盖面向机器人的整个半圆
        angle_offsets = [0.0]
        for deg in range(15, 91, 15):
            rad = math.radians(deg)
            angle_offsets.append(rad)
            angle_offsets.append(-rad)
            
        for r in radii:
            for offset in angle_offsets:
                angle = base_angle + offset
                cand_x = target_x + r * math.cos(angle)
                cand_y = target_y + r * math.sin(angle)
                
                # 避开黑名单中的点
                wp_key = (round(cand_x, 1), round(cand_y, 1))
                if wp_key in self.failed_waypoints:
                    continue
                    
                # 必须是已知的空闲点
                if self.get_map_value(cand_x, cand_y) == 0:
                    # 保守检查：确认周围 0.15m 的四个方向也是空闲的，避免贴墙过近
                    is_safe = True
                    for dx, dy in[(-0.6, 0), (0.6, 0), (0, -0.6), (0, 0.6)]:
                        val = self.get_map_value(cand_x + dx, cand_y + dy)
                        if val == -1 or val > 50:
                            is_safe = False
                            break
                            
                    if is_safe:
                        # 估算离机器人的距离，越近越好，以减少路线折返
                        dist_to_robot = math.hypot(cand_x - rx, cand_y - ry)
                        candidates.append((dist_to_robot, cand_x, cand_y))
                        
        # 优先选择离当前机器人位置最近的安全点
        candidates.sort(key=lambda x: x[0])
        
        # 限制只验证前 4 个候选点，绝对防止假死阻塞
        for cand in candidates[:4]:
            _, wp_x, wp_y = cand
            if self.is_reachable(robot_pose_stamped, wp_x, wp_y):
                # ★ 核心要求：计算让机器人恰好面向顾客的偏航角 (Yaw)
                yaw_to_target = math.atan2(target_y - wp_y, target_x - wp_x)
                return (wp_x, wp_y, yaw_to_target)
                
        return None

    def navigate_until_mapped(self, target_x, target_y):
        rospy.loginfo(f"[Explorer] 开始探索并寻找顾客停靠区 X={target_x:.2f}, Y={target_y:.2f}")
        
        self.failed_waypoints.clear()
        self.clear_costmaps()
        stuck_counter = 0 
        
        while not rospy.is_shutdown():
            rx, ry = self.get_robot_pose()
            robot_pose_stamped = self.get_robot_pose_stamped()
            if rx is None or robot_pose_stamped is None:
                rospy.sleep(0.5)
                continue

            distance_to_target = math.hypot(target_x - rx, target_y - ry)

            # =================== 【新增：1.5m 最终停靠逻辑】 ===================
            # 如果远方发现可以停靠的地方，或者逼近后发现了停靠点，直接发动最终冲刺！
            final_goal = self.check_1_5m_stop_point(rx, ry, target_x, target_y, robot_pose_stamped)
            
            if final_goal is not None:
                stop_x, stop_y, stop_yaw = final_goal
                rospy.loginfo(f"在 1.5m 半径内找到绝佳停靠点: X={stop_x:.2f}, Y={stop_y:.2f}。准备最终停靠！")
                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = stop_x
                goal.target_pose.pose.position.y = stop_y
                
                # 设置机器人姿态：面向顾客
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, stop_yaw)
                goal.target_pose.pose.orientation.x = q[0]
                goal.target_pose.pose.orientation.y = q[1]
                goal.target_pose.pose.orientation.z = q[2]
                goal.target_pose.pose.orientation.w = q[3]
                
                self.move_base_client.send_goal(goal)
                
                # 留出充足的最终导航时间
                self.move_base_client.wait_for_result(rospy.Duration(45.0))
                state = self.move_base_client.get_state()
                
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("成功停靠在目标 1.5m 范围内，且已正对顾客！任务完成。")
                    return True
                else:
                    rospy.logwarn("停靠受到干扰 (可能由于动态行人)，点位拉黑，继续重新规划。")
                    bad_key = (round(stop_x, 1), round(stop_y, 1))
                    self.failed_waypoints.add(bad_key)
                    self.clear_costmaps()
                    continue

            # =================== 兜底与异常保护 ===================
            if distance_to_target < 0.3:
                rospy.logwarn("靠得太近仍无法生成安全停靠点，可能存在遮挡盲区，强制跳出以防撞击。")
                return True

            # =================== 原有的扇形中转逼近逻辑 ===================
            candidates = self.generate_spiral_candidates(rx, ry, target_x, target_y)
            
            valid_goal = None
            for cand in candidates:
                cand_dist, wp_x, wp_y, search_angle = cand
                
                wp_key = (round(wp_x, 1), round(wp_y, 1))
                if wp_key in self.failed_waypoints:
                    continue 
                
                if self.is_reachable(robot_pose_stamped, wp_x, wp_y):
                    valid_goal = cand
                    break

            if valid_goal is None:
                rospy.logerr("[Explorer] 扇形区域内无可用路线！尝试清理代价地图挣扎...")
                self.clear_costmaps()
                stuck_counter += 1
                if stuck_counter > 3:
                    return False
                rospy.sleep(2.0)
                continue

            stuck_counter = 0
            best_dist, target_wp_x, target_wp_y, target_angle = valid_goal
            rospy.loginfo(f"[Explorer] 当前还未找到终点，使用扇形中转点向前推进: ({target_wp_x:.2f}, {target_wp_y:.2f})")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = target_wp_x
            goal.target_pose.pose.position.y = target_wp_y
            
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, target_angle)
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            
            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result(rospy.Duration(20.0))
            state = self.move_base_client.get_state()
            
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[Explorer] 成功抵达中转点，等待地图刷新探测新视野...")
                rospy.sleep(1.0)
            else:
                rospy.logwarn(f"[Explorer] 局部规划受阻 (状态码: {state})。加入黑名单，准备重新扇形规划...")
                bad_key = (round(target_wp_x, 1), round(target_wp_y, 1))
                self.failed_waypoints.add(bad_key)
                self.clear_costmaps()
                rospy.sleep(1.0)
            
        return False