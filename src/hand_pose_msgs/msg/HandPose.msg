# 输入图像的时间戳和坐标系。
std_msgs/Header header

# 当前帧是否检测到满足筛选条件的手。
bool detected

# 经过镜像修正后的 Left 或 Right；未检测到时为空字符串。
string handedness
float32 confidence
float32 processing_time_ms

# MediaPipe 的 21 个图像归一化关键点和世界坐标关键点。
hand_pose_msgs/HandLandmark[] landmarks
hand_pose_msgs/HandLandmark[] world_landmarks

# 人体语义关节角，名称与角度一一对应，单位为弧度。
string[] joint_names
float64[] joint_angles
