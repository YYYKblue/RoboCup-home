# RoboCup@Home 机器人比赛代码整理

本仓库用于整理 RoboCup@Home 机器人比赛中个人贡献的代码部分，主要包括导航、视觉识别和语音交互三个部分。代码以功能模块为主，方便后续自己复盘，也方便之后参加比赛的同学快速理解每个文件的作用，并根据实际机器人平台进行二次开发。

本仓库的代码最终修改于2026.05.04.系统为Ubuntu20.04+ROS noetic 机器人配置:相机为Kinect  底盘为EAI

> 说明：当前仓库不是一个完整的 ROS package，而是比赛过程中抽离出来的功能代码集合。使用时需要根据自己的 ROS 工作空间、机器人底盘、相机型号、导航栈配置和语音模块进行集成。

---

## 1. 项目结构

```text
RoboCup@home/
├── navigation/                  # 导航与坐标变换相关代码
│   ├── best_position.py          # 在目标人物周围搜索最佳可达停靠点
│   ├── camera2base.launch        # 发布相机坐标系与底盘坐标系之间的静态 TF
│   ├── camera_to_map.py          # 将相机坐标系下的三维点转换到 map 坐标系
│   ├── explorer.py               # 未提前完整建图时，边探索边靠近目标点
│   ├── goal_calculator.py        # 根据机器人位姿和目标点计算面向目标的导航位姿
│   ├── navigator.py              # 基于 move_base 的固定点导航封装
│   └── 简短说明.txt              # navigation 部分的原始简短说明
│
├── vision/                       # 视觉检测相关代码
│   └── detect_waving_people.py   # 基于 YOLO-Pose 和 Kinect 深度图检测挥手人员并计算三维坐标
│
└── voice/                        # 语音交互相关代码
    ├── get_keyword.py            # 基于拼音和模糊匹配的关键词识别
    └── summer_tts_speaker.py     # SummerTTS 语音合成 ROS 发布接口封装
```

---

## 2. 模块整体思路

这个项目的核心目标是让机器人在 RoboCup@Home 场景中完成“识别人、定位人、靠近人、与人交互”等任务。代码可以按下面的流程组合：

```text
视觉检测挥手人员
        ↓
获得人物在相机坐标系下的三维坐标
        ↓
通过 TF 转换到 map 坐标系
        ↓
在人物周围搜索可达、安全、面向人物的停靠点
        ↓
调用 move_base 导航到目标点
        ↓
通过语音模块播报或交互
```

其中：

- `vision/` 负责“看见人”和“估计人的三维位置”；
- `navigation/` 负责“坐标变换、目标点计算、路径规划和导航”；
- `voice/` 负责“听懂关键词”和“语音播报”。

---

## 3. navigation：导航与坐标变换

### 3.1 `camera2base.launch`

该 launch 文件用于发布相机与机器人底盘之间的静态坐标变换关系，主要包含两段 TF：

1. `base_link -> camera_link`
   - 描述相机相对于机器人底盘的位置和姿态；
   - 当前参数中相机相对底盘大致位于前后方向 `-0.1 m`、高度 `1.35 m` 的位置。

2. `camera_link -> camera_optical_frame`
   - 描述普通相机坐标系到光学坐标系的转换；
   - 对于深度相机、Kinect、RealSense 等设备，光学坐标系和普通相机坐标系方向通常不同，因此这一段 TF 很重要。

使用 `camera_to_map.py` 前，需要先保证这类 TF 已经正确发布，否则相机坐标无法稳定转换到地图坐标系。

---

### 3.2 `camera_to_map.py`

功能：将相机光学坐标系 `camera_optical_frame` 下的三维点 `[x, y, z]` 转换到地图坐标系 `map` 下。

主要类：

```python
CoordinateConverter
```

主要方法：

```python
get_map_coords(camera_point)
```

输入：

```python
camera_point = [x, y, z]
```

输出：

```python
[x_map, y_map, z_map]
```

典型用途：视觉模块检测到挥手人员后，得到的是人相对于相机的三维坐标；导航模块不能直接使用相机坐标，需要先转换到 `map` 坐标系，再用于导航目标点规划。

注意事项：

- 需要提前启动 TF 发布节点，例如 `camera2base.launch`；
- 需要系统中已经存在 `map -> base_link`、`base_link -> camera_link`、`camera_link -> camera_optical_frame` 等变换链路；
- 如果坐标方向明显错误，优先检查光学坐标系、四元数方向和静态 TF 参数。

---

### 3.3 `navigator.py`

功能：对 ROS `move_base` 导航进行简单封装，用于让机器人导航到预设地点。

主要类：

```python
Navigator
```

初始化时传入地点字典，例如：

```python
locations = {
    "door": [[x, y, z], [qx, qy, qz, qw]],
    "table": [[x, y, z], [qx, qy, qz, qw]]
}
```

主要方法：

| 方法 | 作用 |
|---|---|
| `goto(place)` | 根据地点名称导航到预设位置 |
| `set_goal(frame_id, position, orientation)` | 构造 `MoveBaseGoal` 目标点 |
| `go_to_location(location_goal)` | 向 `move_base` 发送目标点，并在失败时重试 |
| `stop()` | 取消当前导航目标 |

特点：

- 每次导航前会调用 `move_base/clear_costmaps` 清理代价地图；
- 导航失败后会自动重试；
- 适合用于“去门口、去桌子、去起点”等已知地点任务。

---

### 3.4 `goal_calculator.py`

功能：根据机器人当前位姿和目标点坐标，计算一个新的导航目标位姿，使机器人停在目标点前方一定距离，并且朝向目标点。

核心函数：

```python
calculate_facing_goal(robot_pose, target_point_map, distance=0.3)
```

输入：

- `robot_pose`：机器人当前位姿；
- `target_point_map`：目标点在 `map` 坐标系下的位置 `[x, y, z]`；
- `distance`：机器人与目标点保持的距离，默认 `0.3 m`。

输出：

```python
[[goal_x, goal_y, goal_z], [qx, qy, qz, qw]]
```

工作逻辑：

1. 根据机器人当前位置和目标点位置计算方向向量；
2. 从目标点沿反方向后退 `distance` 米，得到机器人应该停靠的位置；
3. 计算朝向目标点的 yaw 角；
4. 将 yaw 角转换为四元数，作为导航目标姿态。

适合场景：目标点附近环境较简单时，直接生成一个面向目标的停靠点。

注意：当前函数内部实际按照 `robot_pose.pose.position` 的结构读取机器人位姿，因此集成时更适合传入 `PoseStamped` 类型或包含 `.pose.position` 字段的对象。

---

### 3.5 `best_position.py`

功能：在目标人物周围的安全环形区域内，搜索一个可达的最佳停靠点，并让机器人停靠后面向人物。

主要类：

```python
SmartGoalFinder
```

主要方法：

| 方法 | 作用 |
|---|---|
| `get_robot_pose()` | 通过 TF 获取机器人当前在 `map` 坐标系下的位姿 |
| `validate_goal(start_pose, goal_pose)` | 调用 `/move_base/make_plan` 判断候选点是否可达 |
| `find_best_goal(person_pose_stamped)` | 围绕人物位置搜索可达停靠点 |

搜索策略：

- 以人物为圆心，在 `0.3 m ~ 0.6 m` 的环形范围内搜索候选点；
- 优先从外圈开始搜索，逐渐向内缩小半径；
- 候选点会设置朝向，使机器人最终面向人物；
- 每个候选点都会通过 `/move_base/make_plan` 验证可达性；
- 找到第一个可达点后返回目标点坐标和姿态。

返回格式：

```python
[[goal_x, goal_y, 0.138], [qx, qy, qz, qw]]
```

适合场景：机器人已经知道人物在 `map` 坐标系下的位置，但人物附近可能存在障碍物，需要搜索一个真正可导航的位置。

注意：当前 `find_best_goal()` 中实际按列表方式读取目标人物坐标，即使用 `person_pose_stamped[0]`、`person_pose_stamped[1]`、`person_pose_stamped[2]`。如果后续改成传入 `PoseStamped`，需要同步修改读取方式。

---

### 3.6 `explorer.py`

功能：在未提前完整建图或目标附近未知区域较多的情况下，让机器人边探索、边靠近目标点，并尝试找到目标周围合适的停靠点。（此文件主要是为了解决26年RoboCup@home的餐厅赛项 且已经能较好的实现功能）

主要类：

```python
ProgressiveExplorer
```

主要方法：

| 方法 | 作用 |
|---|---|
| `map_callback(msg)` | 订阅 `/map`，保存占据栅格地图信息 |
| `get_map_value(x, y)` | 查询地图中某一点是空闲、障碍还是未知 |
| `get_robot_pose()` | 获取机器人当前 `map` 坐标 |
| `get_robot_pose_stamped()` | 获取机器人当前 `PoseStamped` 位姿 |
| `is_reachable(start_pose, target_x, target_y)` | 调用 `/move_base/make_plan` 判断目标是否可达 |
| `clear_costmaps()` | 调用 `/move_base/clear_costmaps` 清理代价地图 |
| `generate_spiral_candidates(rx, ry, target_x, target_y)` | 生成扇形/螺旋式中转候选点 |
| `check_1_5m_stop_point(rx, ry, target_x, target_y, robot_pose_stamped)` | 在目标点附近寻找 0.7 m 到 1.5 m 范围内的可停靠点 |
| `navigate_until_mapped(target_x, target_y)` | 主逻辑：边探索边导航到目标附近 |

工作逻辑：

1. 订阅 `/map` 获取当前地图；
2. 获取机器人当前位姿；
3. 检查目标人物周围是否已经存在安全、可达、面向目标的停靠点；
4. 如果有，直接发送最终导航目标；
5. 如果没有，则沿目标方向生成一批扇形中转点；
6. 选择可达中转点向前推进，等待地图刷新；
7. 如果导航失败，将失败点加入黑名单，并清理代价地图后重新规划。

适合场景：

- 目标点附近地图还没有完全探索；
- 机器人需要在未知环境中逐步靠近顾客或目标人物；
- 直接导航到目标附近容易失败，需要中转点辅助。

---

## 4. vision：挥手人员检测

### `detect_waving_people.py`

该文件目前有两个问题 ：
1 当目标在图像中过小时，yolo的关节模型检测不到，这个问题主要是因为yolo在推理时会缩小图片，所以解决方法是指定imgsz。

2 有些地方深度相机无法获取深度，导致无法计算坐标，这个问题代码中已经尝试解决，依次使用胸口 肩膀 鼻子的深度，但是也可能出现都没有深度的情况。

功能：使用 Kinect 获取彩色图和深度图，通过 YOLO-Pose 检测人体关键点，判断是否有人挥手，并计算挥手人员在相机坐标系下的三维坐标。

主要类：

```python
KinectCamera
WavingPersonDetector
```

### 4.1 `KinectCamera`

用于封装 Azure Kinect 相机相关操作。

主要方法：

| 方法 | 作用 |
|---|---|
| `open_camera()` | 初始化并打开 Kinect 相机 |
| `get_synchronized_frames()` | 获取同步的彩色图和深度图 |
| `get_calibration()` | 返回相机内参矩阵 |
| `release()` | 释放相机资源 |

当前相机内参 `K_kinect` 是硬编码的，如果换相机或更改分辨率，需要重新标定或读取对应内参。

### 4.2 `WavingPersonDetector`

用于检测挥手人员并返回其三维坐标。

主要方法：

| 方法 | 作用 |
|---|---|
| `is_waving(keypoints, conf_threshold=0.7)` | 根据手腕是否高于肩膀判断是否挥手 |
| `get_target_distance_robust(depth_image, center_x, center_y, window_size=40)` | 在目标点附近取深度中值，减少深度噪声影响 |
| `capture_and_detect(camera, max_distance=10.0, save_annotated=True)` | 拍摄一帧图像，检测所有挥手人员，返回按距离排序的结果 |

检测逻辑：

1. 通过 Kinect 获取彩色图和深度图；
2. 使用 YOLO-Pose 检测人体框和关键点；
3. 判断左右手腕是否高于对应肩膀；
4. 对挥手人员估计深度；
5. 根据相机内参进行反投影，得到三维坐标；
6. 按距离从近到远排序；
7. 可选保存带检测框、ID 和距离标注的图片。

输出示例：

```python
[
    {
        "id": 1,
        "x": 0.12,
        "y": -0.03,
        "z": 2.45,
        "distance": 2.46
    }
]
```

其中 `x, y, z` 是相机坐标系下的三维坐标，后续如果要用于导航，需要再通过 `camera_to_map.py` 转换到 `map` 坐标系。

依赖模型：

```text
./model/yolo11m-pose.pt
```

该模型文件没有包含在当前代码目录中，使用前需要自行放置到对应路径，或者修改 `model_path`。

---

## 5. voice：语音关键词与语音播报

### 5.1 `get_keyword.py`

语音识别模块用的是VOSK语音识别，在官网进行下载配置即可，简单易用。（中文识别效果很好 英文识别效果稍差 可以找代替 目前我认为最好的选择是接大模型的api 使用在线语音识别 ）

功能：基于拼音转换和模糊字符串匹配，从语音识别文本中提取目标关键词。主要用于处理语音识别中的错别字、同音字和噪声文本。

主要类：

```python
FuzzyKeywordMatcher
```

主要方法：

```python
find_best_match(text, min_confidence=0.7)
```

输入：

```python
text = "来一包乐是薯片谢谢"
```

输出：

```python
("乐事薯片", 0.85)
```

核心思路：

1. 将目标关键词转换成无声调拼音；
2. 将语音识别文本也转换成拼音；
3. 使用滑动窗口截取候选子串；
4. 使用 `fuzzywuzzy` 计算拼音相似度；
5. 返回相似度最高且超过阈值的关键词。

适合场景：

- 语音识别结果中存在同音错字，例如“可乐”识别成“可了”；
- 环境嘈杂，识别结果中夹杂无关词；
- 商品名、地点名、任务关键词数量有限，可以提前维护关键词列表。

安装依赖：

```bash
pip install pypinyin fuzzywuzzy[speedup]
```

---

### 5.2 `summer_tts_speaker.py`

功能：封装 SummerTTS 的 ROS 发布接口，将文本发布到 `/summer_tts_topic`，触发语音合成节点播报。

语音合成模块主要参考[zhahoi/summer_tts: 在ROS中使用SummerTTS进行语音合成。SummerTTS 是一个独立编译的语音合成程序(TTS)。可以本地运行不需要网络，而且没有额外的依赖，一键编译完成即可用于中文和英文的语音合成。](https://github.com/zhahoi/summer_tts)大家用到的可以过去给个star 做的很好 是一个很不错的离线语音合成工具

主要类：

```python
SummerTTSSpeaker
```

主要方法：

```python
speak(text)
```

使用示例：

```python
speaker = SummerTTSSpeaker()
speaker.speak("我已经找到目标人物")
```

注意事项：

- 使用前需要先启动 SummerTTS 对应的 ROS 节点；
- `speak()` 是非阻塞发送，发布消息后不会等待语音播放完成；
- 如果后续动作依赖语音播报完成，需要手动加入 `rospy.sleep()` 或设计播报完成反馈机制；
- 当前节点发布的话题为 `/summer_tts_topic`，需要与实际 TTS 节点订阅话题保持一致。

---

## 6. 可能需要的依赖

### Python 依赖

```bash
pip install numpy opencv-python torch ultralytics pykinect_azure pypinyin fuzzywuzzy[speedup]
```

### ROS 相关依赖

代码中主要用到了以下 ROS 包或消息类型：

```text
rospy
actionlib
tf2_ros
tf2_geometry_msgs
tf_conversions
geometry_msgs
nav_msgs
move_base_msgs
std_srvs
std_msgs
```

同时需要机器人系统中已经配置好（此部分用的EAI官方封装的相应launch文件）：

- `move_base`；
- `/map` 地图话题；
- `/move_base/make_plan` 路径规划服务；
- `/move_base/clear_costmaps` 代价地图清理服务；
- `map`、`base_link`、`camera_link`、`camera_optical_frame` 等 TF 坐标系。

---

## 7. 使用前建议检查的地方

1. **相机内参是否正确**  
   `detect_waving_people.py` 中的 Kinect 内参是硬编码的，换相机、换分辨率或换深度模式后需要重新确认。

2. **YOLO-Pose 模型路径是否存在**  
   默认模型路径是：

   ```text
   ./model/yolo11m-pose.pt
   ```

3. **TF 是否连通**  
   坐标转换依赖完整 TF 链路，建议使用：

   ```bash
   rosrun tf view_frames
   rosrun tf tf_echo map camera_optical_frame
   ```

4. **导航服务是否正常**  
   `best_position.py` 和 `explorer.py` 都依赖 `/move_base/make_plan` 判断目标点是否可达。

5. **地图和代价地图是否稳定**  
   如果频繁导航失败，可以检查局部代价地图膨胀半径、障碍物层、清图服务是否正常。

6. **语音模块是否提前启动**  
   `summer_tts_speaker.py` 只负责向话题发文本，不负责启动 TTS 节点。

---

## 9. 文件功能速查表

| 文件 | 主要作用 |
|---|---|
| `navigation/camera2base.launch` | 发布相机到机器人底盘、相机到光学坐标系的静态 TF |
| `navigation/camera_to_map.py` | 将相机坐标系下的三维点转换到 `map` 坐标系 |
| `navigation/navigator.py` | 对 `move_base` 进行封装，实现固定地点导航和失败重试 |
| `navigation/goal_calculator.py` | 根据机器人当前位置和目标点生成一个面向目标的导航位姿 |
| `navigation/best_position.py` | 在人物周围搜索可达、安全、面向人物的最佳停靠点 |
| `navigation/explorer.py` | 在未知或半未知地图中边探索边靠近目标点 |
| `vision/detect_waving_people.py` | 使用 Kinect + YOLO-Pose 检测挥手人员，并估计其相机坐标系三维位置 |
| `voice/get_keyword.py` | 使用拼音和模糊匹配从语音识别文本中提取关键词 |
| `voice/summer_tts_speaker.py` | 向 SummerTTS ROS 节点发送文本，触发语音播报 |

