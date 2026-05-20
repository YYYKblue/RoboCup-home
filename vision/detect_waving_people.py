import numpy as np
import cv2
import torch
import time
import os  # 新增：用于操作文件夹
from datetime import datetime  # 新增：用于生成时间戳文件名
from ultralytics import YOLO

class KinectCamera:
    def __init__(self):
        self.K_kinect = np.array([915.0828247070312, 0.0, 961.7936401367188, 
                                  0.0, 914.6190185546875, 555.453369140625, 
                                  0.0, 0.0, 1.0]).reshape(3,3)
        self.device = None
    
    def open_camera(self):
        import pykinect_azure as pykinect
        from pykinect_azure import K4A_FRAMES_PER_SECOND_30, K4A_WIRED_SYNC_MODE_STANDALONE
        pykinect.initialize_libraries()
        device_config = pykinect.default_configuration
        device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_MJPG
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
        device_config.camera_fps = K4A_FRAMES_PER_SECOND_30
        device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE
        device_config.synchronized_images_only = True
        self.device = pykinect.start_device(config=device_config)
        
    def get_synchronized_frames(self):
        if self.device is None: return False, None, None
        capture = self.device.update()
        ret_c, color = capture.get_color_image()
        ret_d, depth = capture.get_transformed_depth_image()
        return (ret_c and ret_d), color, depth
    
    def get_calibration(self):
        return self.K_kinect
        
    def release(self):
        if self.device is not None:
            self.device.stop_cameras()
            self.device.close()


class WavingPersonDetector:
    def __init__(self, model_path='./model/yolo11m-pose.pt'):
        # 核心：必须使用带有关键点检测的 -pose 模型
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        
        # 新增：保存照片的目录设置
        self.photo_dir = "./photo/waving_people"
        os.makedirs(self.photo_dir, exist_ok=True)
        print(f"[初始化] 成功，保存照片的目录是: {self.photo_dir}")

    def is_waving(self, keypoints, conf_threshold=0.7):
        """逻辑判定：检查手腕的关键点Y坐标是否高于肩膀的Y坐标"""
        if keypoints is None or len(keypoints) < 11:
            return False
            
        l_shoulder, r_shoulder = keypoints[5], keypoints[6]
        l_wrist, r_wrist = keypoints[9], keypoints[10]
        
        if r_wrist[2] > conf_threshold and r_shoulder[2] > conf_threshold:
            if r_wrist[1] < r_shoulder[1]: return True
                
        if l_wrist[2] > conf_threshold and l_shoulder[2] > conf_threshold:
            if l_wrist[1] < l_shoulder[1]: return True
                
        return False

    # def get_target_distance_robust(self, depth_image, center_x, center_y, window_size=16):
    #     if depth_image is None: return None
    #     h, w = depth_image.shape
    #     x, y = int(center_x), int(center_y)
    #     x_min, x_max = max(0, x - window_size // 2), min(w, x + window_size // 2)
    #     y_min, y_max = max(0, y - window_size // 2), min(h, y + window_size // 2)
    #     depth_region = depth_image[y_min:y_max, x_min:x_max]
    #     valid_depths = depth_region[depth_region > 0]
    #     if len(valid_depths) == 0: return None  
    #     return np.median(valid_depths) * 0.001 

    def get_target_distance_robust(self, depth_image, center_x, center_y, window_size=40):
        if depth_image is None: return None
        h, w = depth_image.shape
        x, y = int(center_x), int(center_y)
        x_min, x_max = max(0, x - window_size // 2), min(w, x + window_size // 2)
        y_min, y_max = max(0, y - window_size // 2), min(h, y + window_size // 2)
        depth_region = depth_image[y_min:y_max, x_min:x_max]
        valid_depths = depth_region[depth_region > 0]
        if len(valid_depths) == 0: return None  
        return np.median(valid_depths) * 0.001

    # def capture_and_detect(self, camera, max_distance=10.0, save_annotated=True):
    #     """
    #     主调函数：拍一张照片，返回所有挥手人员的排序列表。
    #     save_annotated=True：一旦检测到有人，就保存带标注的照片。
    #     """
    #     ret, color_frame, depth_image = camera.get_synchronized_frames()
    #     if not ret: 
    #         print("警告：相机未能正常获取同步画面。")
    #         return []

    #     # 执行推理， verbose=False 关闭终端刷屏输出
    #     results = self.model(color_frame, verbose=False)
    #     K = camera.get_calibration()
    #     detected_waving_people = []
        
    #     # 用于绘图的变量
    #     annotated_frame = None

    #     if len(results[0].boxes) > 0 and results[0].keypoints is not None:
    #         boxes = results[0].boxes
    #         keypoints_data = results[0].keypoints.data 
            
    #         # 如果需要保存图片，先绘制基础的 YOLO 结果（如骨骼、原检测框）
    #         if save_annotated:
    #             annotated_frame = results[0].plot()

    #         for i in range(len(boxes)):
    #             if int(boxes.cls[i]) != 0 or boxes.conf[i] < 0.7: continue
                    
    #             person_keypoints = keypoints_data[i].cpu().numpy()
    #             if not self.is_waving(person_keypoints): continue
                    
    #             # 计算人体的中心点（取边界框中心即可）
    #             x1, y1, x2, y2 = map(int, boxes.xyxy[i])
    #             center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
                
    #             # 计算深度
    #             z_depth = self.get_target_distance_robust(depth_image, center_x, center_y)
    #             if not z_depth or z_depth > max_distance: continue
                    
    #             # 逆投影计算 3D 坐标
    #             point_2d = np.array([center_x, center_y, 1])
    #             point_3d = z_depth * np.linalg.inv(K).dot(point_2d)
    #             # 计算直线距离 (欧氏距离)
    #             straight_distance = np.linalg.norm(point_3d)
                
    #             detected_waving_people.append({
    #                 'coords': (float(point_3d[0]), float(point_3d[1]), float(point_3d[2])),
    #                 'distance': float(straight_distance),
    #                 'box_top_left': (x1, y1) # 用于后面绘图
    #             })

    #     # 按直线距离由近到远排序
    #     detected_waving_people.sort(key=lambda p: p['distance'])
        
    #     # 分配 ID 并生成最终结果
    #     final_results = []
    #     for index, person in enumerate(detected_waving_people):
    #         person_id = index + 1
    #         final_results.append({
    #             'id': person_id,
    #             'x': person['coords'][0],
    #             'y': person['coords'][1],
    #             'z': person['coords'][2],
    #             'distance': person['distance']
    #         })
            
    #         # 新增保存逻辑：在图片上绘制ID和距离
    #         if save_annotated and annotated_frame is not None:
    #             x, y = person['box_top_left']
    #             # 绘制蓝底白字的ID和距离，显眼一些
    #             text = f"WAVE ID:{person_id} | {person['distance']:.2f}m"
    #             cv2.putText(annotated_frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
    #                         0.7, (255, 0, 0), 2)
    #             # 在画面左上角显眼位置打上“检测成功”标签
    #             cv2.putText(annotated_frame, "STATUS: WAVING DETECTED", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 
    #                         1.2, (0, 0, 255), 3)

    #     # 最终执行照片保存
    #     if save_annotated and len(final_results) > 0 and annotated_frame is not None:
    #         time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    #         save_path = os.path.join(self.photo_dir, f"wave_{time_str}.jpg")
    #         # 确保文件夹还在（有些系统会定时清理/tmp等临时文件夹）
    #         os.makedirs(self.photo_dir, exist_ok=True)
    #         if cv2.imwrite(save_path, annotated_frame):
    #             print(f"[系统] 已成功保存一张检测照片到: {save_path}")
    #         else:
    #             print(f"[错误] 照片保存失败，尝试保存路径: {save_path}")
            
    #     return final_results
    def capture_and_detect(self, camera, max_distance=10.0, save_annotated=True):
        """
        主调函数：拍一张照片，返回所有挥手人员的排序列表。
        save_annotated=True：一旦检测到有人，就保存带标注的照片。
        """
        ret, color_frame, depth_image = camera.get_synchronized_frames()
        if not ret: 
            print("警告：相机未能正常获取同步画面。")
            return[]

        # 执行推理， verbose=False 关闭终端刷屏输出
        results = self.model(color_frame, verbose=False)
        K = camera.get_calibration()
        detected_waving_people =[]
        
        # 用于绘图的变量
        annotated_frame = None

        if len(results[0].boxes) > 0 and results[0].keypoints is not None:
            boxes = results[0].boxes
            keypoints_data = results[0].keypoints.data 
            
            # 如果需要保存图片，先绘制基础的 YOLO 结果（如骨骼、原检测框）
            if save_annotated:
                # 🌟 修复1：加上 labels=False 关闭默认的 "person 0.90" 标签，防止与自定义文字重叠
                annotated_frame = results[0].plot(labels=False)

            for i in range(len(boxes)):
                if int(boxes.cls[i]) != 0 or boxes.conf[i] < 0.7: continue
                    
                person_keypoints = keypoints_data[i].cpu().numpy()
                if not self.is_waving(person_keypoints): continue
                    
                # 计算人体的中心点（取边界框中心即可）
                # ------------------- 核心坐标提取逻辑优化 -------------------
                x1, y1, x2, y2 = map(int, boxes.xyxy[i])
                
                # YOLO-pose 关键点索引: 0=鼻子, 5=左肩, 6=右肩
                nose = person_keypoints[0]
                l_shoulder = person_keypoints[5]
                r_shoulder = person_keypoints[6]
                
                # 优先级 1：面部（鼻子）。人脸皮肤对红外深度反射好，且极少被遮挡
                if nose[2] > 0.5:
                    center_x = nose[0]
                    center_y = nose[1]
                    
                # 优先级 2：双肩中点（胸口/脖子处）。防止人背对镜头或低头导致鼻子没测到
                elif l_shoulder[2] > 0.5 and r_shoulder[2] > 0.5:
                    center_x = (l_shoulder[0] + r_shoulder[0]) / 2
                    center_y = (l_shoulder[1] + r_shoulder[1]) / 2
                    
                # 优先级 3：退回边界框（稍微偏上部一点，避开腿部和抬起的手臂）
                else:
                    center_x = (x1 + x2) / 2
                    center_y = y1 + (y2 - y1) * 0.3  
                
                # 计算深度 (保留较大的 window_size=40 以包容面部的起伏或头发边缘)
                z_depth = self.get_target_distance_robust(depth_image, center_x, center_y, window_size=40)
                # -----------------------------------------------------------
                if not z_depth or z_depth > max_distance: continue
                    
                # 逆投影计算 3D 坐标
                point_2d = np.array([center_x, center_y, 1])
                point_3d = z_depth * np.linalg.inv(K).dot(point_2d)
                # 计算直线距离 (欧氏距离)
                straight_distance = np.linalg.norm(point_3d)
                
                detected_waving_people.append({
                    'coords': (float(point_3d[0]), float(point_3d[1]), float(point_3d[2])),
                    'distance': float(straight_distance),
                    'box_top_left': (x1, y1) # 用于后面绘图
                })

        # 按直线距离由近到远排序
        detected_waving_people.sort(key=lambda p: p['distance'])
        
        # 分配 ID 并生成最终结果
        final_results =[]
        for index, person in enumerate(detected_waving_people):
            person_id = index + 1
            final_results.append({
                'id': person_id,
                'x': person['coords'][0],
                'y': person['coords'][1],
                'z': person['coords'][2],
                'distance': person['distance']
            })
            
            # 新增保存逻辑：在图片上绘制ID和距离
            if save_annotated and annotated_frame is not None:
                x, y = person['box_top_left']
                
                # 🌟 修复2：实现真正的“蓝底白字”，且防止文字超出图片顶部出界
                text = f"WAVE ID:{person_id} | {person['distance']:.2f}m"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                thickness = 2
                
                # 获取文字尺寸，用于计算背景框大小
                (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
                
                # 避免检测框太靠上导致标签画到图片外面去
                if y - text_h - 15 < 0:
                    y = y + text_h + 20 
                
                # 绘制蓝色的实心背景框：OpenCV颜色是(B,G,R)，所以(255,0,0)为纯蓝色 (-1代表实心填充)
                cv2.rectangle(annotated_frame, 
                              (x, y - text_h - 15), 
                              (x + text_w + 10, y + baseline - 5), 
                              (255, 0, 0), -1)
                
                # 在蓝底上绘制白色文字：(255,255,255)
                cv2.putText(annotated_frame, text, (x + 5, y - 10), font, 
                            font_scale, (255, 255, 255), thickness)

        # 🌟 修复3：将总状态标签移到循环外，避免多个人时文字叠加导致边缘锯齿加深
        if save_annotated and annotated_frame is not None and len(final_results) > 0:
            cv2.putText(annotated_frame, "STATUS: WAVING DETECTED", (30, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        # 最终执行照片保存
        if save_annotated and len(final_results) > 0 and annotated_frame is not None:
            time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.photo_dir, f"wave_{time_str}.jpg")
            # 确保文件夹还在（有些系统会定时清理/tmp等临时文件夹）
            os.makedirs(self.photo_dir, exist_ok=True)
            if cv2.imwrite(save_path, annotated_frame):
                print(f"[系统] 已成功保存一张检测照片到: {save_path}")
            else:
                print(f"[错误] 照片保存失败，尝试保存路径: {save_path}")
            
        return final_results

# ==============================================================================
# 测试代码区域：直接运行 python detect_waving_people.py 即可触发测试
# ==============================================================================
if __name__ == "__main__":
    print("[测试脚本] 正在初始化相机与模型...")
    camera = KinectCamera()
    
    try:
        camera.open_camera()
        # 注意：此处使用的模型必须带 -pose
        detector = WavingPersonDetector(model_path='./model/yolo11m-pose.pt') 
        
        # 让相机跑几帧预设曝光，防止开局黑屏
        print("[测试脚本] 正在等待硬件曝光稳定...")
        for _ in range(15):
            camera.get_synchronized_frames()
            time.sleep(0.05)
            
        print("\n[测试脚本] ================== 测试开始 ==================")
        print("请站在相机前方 5米内 挥手...")
        
        # 模拟主程序的周期性调用，进行 2 次测试（每次间隔1.5秒）
        for i in range(2):
            print(f"\n---> 正在执行第 {i+1} 次单帧拍摄...")
            
            # 这里就是你在主控节点里调用的那句核心代码。save_annotated 默认为True。
            waving_people = detector.capture_and_detect(camera, max_distance=5.0)
            
            if len(waving_people) == 0:
                print("视野内未检测到有效的挥手人员。")
            else:
                print(f"检测成功，共锁定 {len(waving_people)} 人：")
                for p in waving_people:
                    print(f"  [ID: {p['id']}] - 距离: {p['distance']:.2f}m | 坐标: X={p['x']:.2f}, Y={p['y']:.2f}, Z={p['z']:.2f}")
            
            # 延时 1.5 秒，方便你在终端观察输出和去挥手
            time.sleep(1.5)
                
    except KeyboardInterrupt:
        print("\n[测试脚本] 检测到中断信号，正在退出...")
    except Exception as e:
        print(f"\n[测试脚本] 运行出错: {e}")
    finally:
        print("[测试脚本] 正在释放相机资源...")
        camera.release()
        print("[测试脚本] 退出完成。")