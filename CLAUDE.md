# Drobot 프로젝트 메모

## 프로젝트 개요
- ROS 2 Jazzy + Gazebo Harmonic 기반 로봇 시뮬레이션
- 4바퀴 스키드 스티어 + 쿼드콥터 프로펠러 로봇

## 현재 상태
- [x] 기본 URDF 완성 (단순 도형 사용)
- [x] Gazebo 플러그인 설정 완료 (DiffDrive, LiDAR, Camera, IMU)
- [x] SLAM, Navigation 런치 파일 준비됨
- [ ] SolidWorks STL 메시 적용 안됨

## TODO
1. **SolidWorks에서 STL export**
   - 파일: `drobot v0.0.sldprt`
   - export 후 `drobot_simulation/meshes/`에 저장

2. **URDF에 STL 적용**
   - visual에만 STL 사용
   - collision은 단순 도형 유지 (성능 이슈 방지)

3. **메시 최적화 (필요시)**
   - 폴리곤 수 줄이기
   - 여러 파트 하나로 합치기

## 성능 관련 참고
- 파트가 많거나 메시가 복잡하면 Gazebo 렉 발생
- 해결책: Visual은 상세 STL, Collision은 단순 박스 사용

## 주요 명령어
```bash
# 시뮬레이션 실행
ros2 launch drobot_simulation simulation.launch.py

# 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

## 파일 구조
```
drobot_simulation/
├── urdf/drobot.urdf.xacro    # 로봇 모델
├── meshes/                    # STL 파일 넣을 곳
├── worlds/                    # Gazebo 월드들
├── launch/                    # 런치 파일들
└── config/                    # 설정 파일들
```
