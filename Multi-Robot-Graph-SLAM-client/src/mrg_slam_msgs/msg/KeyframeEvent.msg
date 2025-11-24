# KeyframeEvent.msg
# 로봇에서 서버로 전송되는 키프레임 이벤트
# B안: 로컬 SLAM 결과를 간헐적으로 업로드

std_msgs/Header header

# 로봇 식별
string robot_name

# 키프레임 식별자
string keyframe_uuid

# 누적 이동 거리 (키프레임 생성 기준)
float64 accum_distance

# 키프레임 포즈 (odom 프레임 기준)
geometry_msgs/Pose estimate

# 다운샘플된 포인트 클라우드 (업로드용, 0.5m 권장)
sensor_msgs/PointCloud2 cloud

# 선택: 이전 키프레임과의 엣지 정보
# EdgeInfo edge_from_prev