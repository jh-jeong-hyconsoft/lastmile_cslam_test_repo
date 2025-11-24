# B-Plan KeyframeUploader C++ 컴포넌트 업데이트

## 변경 사항 요약

### 배경
Python `keyframe_uploader_node.py`를 C++ `KeyframeUploaderComponent`로 교체하면서 토픽 이름이 변경되었습니다.

**변경된 토픽 이름:**
- **이전**: `/robot1/slam/keyframe_event` (MrgSlamComponent 직접 발행)
- **이후**: `/robot1/slam/keyframe_upload` (KeyframeUploaderComponent 재발행)

---

## 수정된 파일 목록

### 1. C++ 컴포넌트
- `src/mrg_slam/apps/keyframe_uploader_component.cpp` (신규)
  - Rate-limited keyframe upload (0.3 Hz)
  - 구독: `slam/keyframe_event` (내부, MrgSlamComponent 출력)
  - 발행: `slam/keyframe_upload` (외부, Zenoh Bridge로 전송)

### 2. 서버 측 SLAM 컴포넌트
- `src/mrg_slam/apps/mrg_slam_component.cpp`
  - **파라미터 추가**:
    - `enable_keyframe_event_reception` (bool, default: false)
  - **구독 추가**:
    - `slam/keyframe_upload` 토픽 (서버 모드에서만)
  - **콜백 추가**:
    - `keyframe_event_callback()`: 수신한 KeyframeEvent를 그래프에 추가

### 3. Zenoh Bridge 설정 (Jetson B)
- `deployment/jetson_b/configs/zenoh_bridge_b_plan_corrected.json5`
  - QoS 토픽: `/robot1/slam/keyframe_upload`
  - allow.publishers: `/robot1/slam/keyframe_upload`

### 4. Zenoh Bridge 설정 (서버)
- `deployment/server/zenoh/robot1.bridge.json5`
  - QoS 토픽: `/robot1/slam/keyframe_upload`
  - allow.subscribers: `/robot1/slam/keyframe_upload`

- `deployment/server/zenoh/robot2.bridge.json5`
  - QoS 토픽: `/robot2/slam/keyframe_upload`
  - allow.subscribers: `/robot2/slam/keyframe_upload`

- `deployment/server/zenoh/robot3.bridge.json5`
  - QoS 토픽: `/robot3/slam/keyframe_upload`
  - allow.subscribers: `/robot3/slam/keyframe_upload`

### 5. 서버 Launch 파일
- `deployment/server/launch/mrg_slam_backend_b_plan_with_merger.launch.py`
  - 파라미터 추가: `enable_keyframe_event_reception: True`

---

## 데이터 흐름 (업데이트)

```
┌─────────────────────────────────┐
│ Jetson A (로봇)                  │
│                                 │
│  센서 → SLAM → KeyFrame 생성     │
│                    ↓             │
│         MrgSlamComponent         │
│          (keyframe_event)        │
│                    ↓             │
│      KeyframeUploaderComponent   │  ← Rate Limiter (0.3 Hz)
│         (keyframe_upload)        │
└────────────────────┼─────────────┘
                     │
┌────────────────────┼─────────────┐
│ Jetson B           ↓              │
│           Zenoh Bridge            │
│   /robot1/slam/keyframe_upload   │
└────────────────────┼─────────────┘
                     │ VPN (WireGuard)
                     ↓
┌────────────────────┼─────────────┐
│ 서버               ↓              │
│           Zenoh Bridge            │
│   /robot1/slam/keyframe_upload   │
│                    ↓              │
│        MrgSlamComponent           │
│    (keyframe_event_callback)     │
│                    ↓              │
│         그래프에 KeyFrame 추가    │
└───────────────────────────────────┘
```

---

## 토픽 구조

### Jetson A (robot1)
- **발행**:
  - `/robot1/slam/keyframe_event` (내부, localhost)
  - `/robot1/slam/keyframe_upload` (외부, Zenoh로 전송)
  - `/robot1/mrg_slam/slam_status`

- **구독**:
  - `/robot1/mrg_slam/slam_pose_broadcast` (서버로부터)

### 서버 (robot1_backend)
- **구독**:
  - `/robot1/slam/keyframe_upload` (Zenoh로부터)
  - `/robot1/mrg_slam/slam_status`

- **발행**:
  - `/robot1/mrg_slam/slam_pose_broadcast`
  - `/robot1/mrg_slam/map_points`

---

## 빌드 및 테스트

### 1. 빌드
```bash
cd ~/ros2_ws

# mrg_slam 패키지 재빌드
colcon build --packages-select mrg_slam

source install/setup.bash
```

### 2. 컴포넌트 확인
```bash
# KeyframeUploaderComponent 확인
ros2 component types | grep KeyframeUploader
# 예상: mrg_slam  mrg_slam::KeyframeUploaderComponent

# MrgSlamComponent 파라미터 확인
ros2 param describe /robot1/robot1_backend enable_keyframe_event_reception
```

### 3. 토픽 흐름 테스트

#### Jetson A
```bash
# 런치
ros2 launch deployment/jetson_a/launch/robot_full_b_plan.launch.py \
    config:=robot1_front.yaml

# 토픽 확인
ros2 topic list | grep keyframe
# 예상:
# /robot1/slam/keyframe_event
# /robot1/slam/keyframe_upload

# KeyframeUploader 로그 확인
ros2 topic echo /rosout | grep -i "uploaded keyframe"
```

#### 서버
```bash
# 런치
ros2 launch deployment/server/launch/mrg_slam_backend_b_plan_with_merger.launch.py

# 수신 확인
ros2 topic echo /robot1/slam/keyframe_upload --once

# 로그 확인
ros2 topic echo /rosout | grep -i "received keyframe event"
```

---

## 중요 참고사항

### 1. 토픽 네이밍
- **내부 (localhost)**: `slam/keyframe_event`
- **외부 (Zenoh)**: `slam/keyframe_upload`
- 이유: 무한 루프 방지, rate limiting 적용

### 2. 서버 모드 활성화
서버의 `MrgSlamComponent`는 반드시 다음 파라미터를 설정해야 합니다:
- `enable_keyframe_event_publish: false` (발행 비활성화)
- `enable_keyframe_event_reception: true` (수신 활성화)

### 3. 로봇 모드 설정
로봇의 `MrgSlamComponent`는:
- `enable_keyframe_event_publish: true` (발행 활성화)
- `enable_keyframe_event_reception: false` (수신 비활성화)

### 4. Zenoh Bridge 네임스페이스
- **로봇 측**: `namespace: "/robot1"` 설정
- **서버 측**: `namespace` 없음 (이미 `/robot1/` 붙어서 들어옴)

---

## 트러블슈팅

### 문제 1: 서버에서 KeyframeEvent 수신 안됨
**원인**: Zenoh Bridge의 토픽 이름 불일치
**해결**:
```bash
# 로봇 측 발행 확인
ros2 topic list | grep keyframe_upload

# 서버 측 Zenoh Bridge 로그 확인
docker logs zenoh-bridge-robot1 | grep keyframe
```

### 문제 2: 서버에 키프레임이 추가되지 않음
**원인**: `enable_keyframe_event_reception` 파라미터 누락
**해결**:
```bash
# 서버 파라미터 확인
ros2 param get /robot1/robot1_backend enable_keyframe_event_reception

# true가 아니면 launch 파일 수정 필요
```

### 문제 3: Rate limiting이 작동하지 않음
**원인**: KeyframeUploaderComponent 실행 안됨
**해결**:
```bash
# 컴포넌트 확인
ros2 component list

# 예상 출력:
# /robot1/mrg_slam_full_container
#   1  /robot1/prefiltering
#   2  /robot1/scan_matching_odometry
#   3  /robot1/mrg_slam_component
#   4  /robot1/keyframe_uploader
```

---

## 다음 단계

1. **빌드**: `colcon build --packages-select mrg_slam`
2. **Jetson A 테스트**: KeyframeUploader 로그 확인
3. **Zenoh Bridge 설정**: robot1/2/3 설정 동일하게 적용
4. **서버 테스트**: KeyframeEvent 수신 및 그래프 추가 확인
5. **통합 테스트**: 전체 시스템 end-to-end 테스트

---

**작성일**: 2025-11-14
**버전**: B-Plan v2 (C++ KeyframeUploader)

