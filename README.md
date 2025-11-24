# Multi-Robot SLAM Deployment (B-Plan v2)

**최종 업데이트**: 2025-11-14  
**버전**: B-Plan v2 (C++ KeyframeUploader)  
**상태**: Production Ready

---

## 문서 구조

### 핵심 문서 (시작하기)

1. **[DEPLOYMENT_CHECKLIST.md](./DEPLOYMENT_CHECKLIST.md)** ⭐
   - 단계별 배포 체크리스트
   - Phase 1-6 완전 가이드
   - 최종 검증 항목
   - **권장**: 처음 시작할 때 이 문서 따라하기

2. **[FINAL_DEPLOYMENT_GUIDE.md](./FINAL_DEPLOYMENT_GUIDE.md)** ⭐⭐⭐
   - 완전한 배포 매뉴얼 (200+ 페이지)
   - 시스템 아키텍처 설명
   - 각 디바이스별 상세 가이드
   - 트러블슈팅 포함
   - **권장**: 상세한 이해가 필요할 때 참조

### 빠른 참조 가이드 (각 디바이스별)

3. **[jetson_a/DEPLOYMENT_QUICK_GUIDE.md](./jetson_a/DEPLOYMENT_QUICK_GUIDE.md)**
   - Jetson A (로봇 메인 PC) 빠른 배포
   - 5분 안에 실행 가능
   - 필수 설정만 포함

4. **[jetson_b/DEPLOYMENT_QUICK_GUIDE.md](./jetson_b/DEPLOYMENT_QUICK_GUIDE.md)**
   - Jetson B (브리지 PC) 빠른 배포
   - WireGuard + Zenoh Bridge
   - Systemd 서비스 설정

5. **[server/DEPLOYMENT_QUICK_GUIDE.md](./server/DEPLOYMENT_QUICK_GUIDE.md)**
   - 서버 (중앙 서버) 빠른 배포
   - Docker Compose 원클릭 실행
   - RViz 시각화 설정

### 소스 코드 빌드 가이드 (디바이스별)

6. **[SOURCE_COMPARISON.md](./SOURCE_COMPARISON.md)** ⭐⭐
   - **디바이스별 소스 구성 비교표**
   - CMakeLists.txt 차이
   - 빌드 명령 및 리소스 비교
   - **권장**: 빌드 전 반드시 읽기

7. **[jetson_a/SOURCE_BUILD_GUIDE.md](./jetson_a/SOURCE_BUILD_GUIDE.md)**
   - Jetson A 전체 패키지 빌드
   - 의존성 설치 및 검증
   - 트러블슈팅

8. **[jetson_b/SOURCE_BUILD_GUIDE.md](./jetson_b/SOURCE_BUILD_GUIDE.md)**
   - Jetson B 메시지만 빌드
   - 최소 의존성 설치
   - Zenoh-Bridge 설치

9. **[server/SOURCE_BUILD_GUIDE.md](./server/SOURCE_BUILD_GUIDE.md)**
   - 서버 Docker 기반 빌드
   - Dockerfile 및 Docker Compose
   - 이미지 배포 방법

### 기술 문서

10. **[B_PLAN_KEYFRAME_UPLOAD_UPDATE.md](./B_PLAN_KEYFRAME_UPLOAD_UPDATE.md)**
    - C++ KeyframeUploader 변경사항
    - 토픽 이름 변경 (`keyframe_event` → `keyframe_upload`)
    - 서버 수신 로직 추가

11. **[ZENOH_TROUBLESHOOTING.md](./ZENOH_TROUBLESHOOTING.md)**
    - Zenoh Bridge 문제 해결
    - 네임스페이스 중복 문제
    - Peer/Client 모드 선택

12. **[ZENOH_BRIDGE_COMPARISON.md](./ZENOH_BRIDGE_COMPARISON.md)**
    - Zenoh Bridge 설정 비교
    - A-Plan vs B-Plan
    - 네트워크 최적화

---

## 시스템 개요

### 아키텍처

```
┌─────────────────────────────────────────────────┐
│ Robot 1, 2, 3 (각각 동일 구조)                   │
│                                                 │
│  [Jetson A] 센서 + 로컬 SLAM                    │
│     ↓ KeyframeEvent (0.3Hz, 500KB)             │
│  [Jetson B] Zenoh Bridge                        │
│     ↓ LTE + WireGuard VPN                       │
└───────────────────┼─────────────────────────────┘
                    │
                    ↓
┌───────────────────┼─────────────────────────────┐
│ Server            ↓                             │
│                                                 │
│  [WireGuard VPN] 10.13.13.1/24                  │
│     ↓                                            │
│  [Zenoh Bridge × 3] :7448, :7449, :7450         │
│     ↓                                            │
│  [SLAM Backend × 3] 전역 최적화                  │
│     ↓                                            │
│  [Global Map Merger] 통합 맵 생성                │
│     ↓                                            │
│  /global/map_merged (world frame)               │
└──────────────────────────────────────────────────┘
```

### 데이터 흐름

1. **센서 → 로컬 SLAM** (Jetson A)
   - LiDAR, IMU 데이터 처리
   - Prefiltering, Scan Matching Odometry
   - 로컬 그래프 최적화

2. **KeyframeEvent 생성** (Jetson A)
   - MrgSlamComponent: KeyframeEvent 발행 (내부)
   - KeyframeUploaderComponent: Rate limiting (0.3 Hz)
   - 다운샘플링 (0.5m → 약 500KB/msg)

3. **네트워크 전송** (Jetson B)
   - Zenoh Bridge: ROS2 → Zenoh
   - WireGuard VPN: 암호화 전송 (LTE)
   - 대역폭: 약 150 KB/s per robot

4. **서버 수신 및 최적화** (Server)
   - Zenoh Bridge: Zenoh → ROS2
   - MrgSlamComponent Backend: KeyframeEvent 수신
   - 전역 그래프 최적화 (로봇 간 루프 클로저)

5. **Global Map 생성** (Server)
   - GlobalMapMergerComponent: 3개 로봇 맵 병합
   - VoxelGrid 다운샘플링 (0.1m)
   - `/global/map_merged` 발행 (world frame, 30초마다)

---

## 빠른 시작 (3단계)

### Step 1: 서버 실행 (5분)

```bash
cd ~/Multi-Robot-Graph-SLAM/deployment/server
docker-compose -f docker/docker-compose.yml up -d

# Backend 시작
docker exec -it ros2-vnc-slam bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 launch /workspace/launch/mrg_slam_backend_b_plan_with_merger.launch.py \
    config:=backend_b_plan.yaml rviz:=false &
```

### Step 2: Robot1 실행 (10분)

**Jetson B (브리지)**:
```bash
sudo wg-quick up wg0  # WireGuard VPN
sudo systemctl start zenoh-bridge-robot1  # Zenoh Bridge
```

**Jetson A (SLAM)**:
```bash
cd ~/deployment_robot1
ros2 launch launch/robot_full_b_plan.launch.py config:=robot1_front.yaml
```

### Step 3: 검증 (2분)

```bash
# 서버에서 (ROS2 컨테이너 내부)
ros2 topic hz /robot1/slam/keyframe_upload      # 0.3 Hz
ros2 topic echo /rosout | grep "Received keyframe"  # 로그 확인
ros2 topic hz /global/map_merged                 # 0.033 Hz
```

**성공!** 🎉

---

## 디렉토리 구조

```
deployment/
├── README.md (이 파일)
│
├── DEPLOYMENT_CHECKLIST.md          ⭐ 체크리스트
├── FINAL_DEPLOYMENT_GUIDE.md        ⭐⭐⭐ 완전 가이드
├── SOURCE_COMPARISON.md             ⭐⭐ 소스 비교표
│
├── B_PLAN_KEYFRAME_UPLOAD_UPDATE.md (기술 문서)
├── ZENOH_TROUBLESHOOTING.md         (트러블슈팅)
├── ZENOH_BRIDGE_COMPARISON.md       (비교 분석)
│
├── jetson_a/                        (로봇 메인 PC)
│   ├── DEPLOYMENT_QUICK_GUIDE.md    ⭐ 빠른 가이드
│   ├── SOURCE_BUILD_GUIDE.md        ⭐ 빌드 가이드
│   ├── configs/
│   │   ├── robot1_front.yaml        (설정 파일)
│   │   ├── robot2_front.yaml
│   │   └── robot3_front.yaml
│   └── launch/
│       └── robot_full_b_plan.launch.py
│
├── jetson_b/                        (브리지 PC)
│   ├── DEPLOYMENT_QUICK_GUIDE.md    ⭐ 빠른 가이드
│   ├── SOURCE_BUILD_GUIDE.md        ⭐ 빌드 가이드
│   └── configs/
│       ├── zenoh_bridge_robot1.json5 (Zenoh 설정)
│       ├── zenoh_bridge_robot2.json5
│       └── zenoh_bridge_robot3.json5
│
└── server/                          (중앙 서버)
    ├── DEPLOYMENT_QUICK_GUIDE.md    ⭐ 빠른 가이드
    ├── SOURCE_BUILD_GUIDE.md        ⭐ 빌드 가이드
    ├── docker/
    │   ├── docker-compose.yml       (Docker Compose)
    │   └── Dockerfile.ros2-vnc
    ├── zenoh/
    │   ├── robot1.bridge.json5      (Zenoh 설정)
    │   ├── robot2.bridge.json5
    │   └── robot3.bridge.json5
    ├── configs/
    │   └── backend_b_plan.yaml      (SLAM 설정)
    ├── launch/
    │   └── mrg_slam_backend_b_plan_with_merger.launch.py
    └── rviz/
        └── global_map_view.rviz
```

---

## 주요 특징

### B-Plan v2 (현재)
- ✅ C++ KeyframeUploader (Python 대비 30% 성능 향상)
- ✅ Rate limiting (0.3 Hz, 대역폭 절약)
- ✅ 서버 KeyframeEvent 수신 로직 (자동 그래프 추가)
- ✅ TF 분리 (SLAM은 TF 발행 안함, RL이 담당)
- ✅ 네임스페이스 완전 분리 (robot1, robot2, robot3)
- ✅ Global Map Merger (C++, 실시간 병합)
- ✅ WireGuard VPN (LTE 암호화 전송)
- ✅ Zenoh Bridge (불안정 네트워크 최적화)
- ✅ Docker Compose (서버 원클릭 배포)

### A-Plan vs B-Plan 비교
| 항목 | A-Plan | B-Plan v2 |
|------|--------|-----------|
| 로컬 SLAM | ❌ | ✅ (Jetson A) |
| 서버 부하 | 높음 | 중간 |
| 대역폭 | 높음 (2-3 MB/s) | 낮음 (150 KB/s) |
| 로봇 자율성 | 낮음 | 높음 |
| 네트워크 장애 | 치명적 | 일시적 |

---

## 요구 사항

### 하드웨어
- **Jetson AGX Orin** × 3 (32GB RAM)
- **브리지 PC** × 3 (Jetson Nano 가능)
- **서버**: 8+ cores, 32GB+ RAM, SSD 500GB+
- **LTE 모뎀** × 3 (WireGuard 지원)
- **LiDAR** × 3 (Velodyne VLP-16 권장)

### 소프트웨어
- **OS**: Ubuntu 22.04 (모든 디바이스)
- **ROS2**: Humble
- **DDS**: CycloneDDS (권장)
- **Docker**: 20.10+
- **WireGuard**: Latest
- **Zenoh-Bridge**: 1.0.4+

### 네트워크
- **대역폭**: 최소 500 KB/s per robot (LTE)
- **지연**: < 200ms (VPN)
- **VPN**: WireGuard 10.13.13.0/24

---

## 트러블슈팅

### 자주 발생하는 문제

1. **KeyframeEvent 발행 안됨**
   - 원인: 초기화 안됨, 파라미터 설정 오류
   - 해결: `FINAL_DEPLOYMENT_GUIDE.md` Section 8.1 참조

2. **Zenoh 연결 실패**
   - 원인: VPN 끊김, 포트 불일치
   - 해결: `ZENOH_TROUBLESHOOTING.md` 참조

3. **서버 수신 안됨**
   - 원인: 토픽 이름 불일치, DDS Domain 불일치
   - 해결: `B_PLAN_KEYFRAME_UPLOAD_UPDATE.md` 참조

4. **MrgSlamComponent 크래시**
   - 원인: g2o solver `lm_var_cholmod` 미지원
   - 해결: `g2o_solver_type: "lm_var"`로 변경

5. **Global Map 생성 안됨**
   - 원인: Backend가 keyframe 못받음
   - 해결: `FINAL_DEPLOYMENT_GUIDE.md` Section 8.5 참조

---

## 지원

### 문서
- **전체 가이드**: `FINAL_DEPLOYMENT_GUIDE.md`
- **체크리스트**: `DEPLOYMENT_CHECKLIST.md`
- **빠른 가이드**: 각 디바이스별 `DEPLOYMENT_QUICK_GUIDE.md`

### 로그 수집
```bash
# Jetson A
journalctl -u mrg-slam-robot1 -n 100 > jetson_a_log.txt

# Jetson B
journalctl -u zenoh-bridge-robot1 -n 100 > jetson_b_log.txt

# 서버
docker logs ros2-vnc-slam --tail 100 > server_log.txt
```

### 디버깅
```bash
# ROS2 토픽 확인
ros2 topic list
ros2 topic hz <topic_name>
ros2 topic echo <topic_name> --once

# 네트워크 확인
ping 10.13.13.1
sudo wg show
sudo iftop -i wg0

# 컨테이너 확인
docker ps
docker logs <container_name>
```

---

## 라이선스

BSD-2-Clause (소스 코드 헤더 참조)

---

## 기여

버그 리포트 및 개선 제안은 GitHub Issues로 제출해주세요.

---

**Happy SLAM-ing!** 🚀🤖
