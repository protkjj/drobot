# Drobot TODO

## 현재 작업: VER1 자율주행 시스템

### VER1 - drobot_navigation (🔄 진행중)
- [x] 패키지 구조 생성
- [x] goal_navigator.py 메인 노드
- [x] 규칙 엔진 (rules/engine.py)
- [x] rules.yaml 규칙 정의 파일
- [x] nav2_params.yaml 정리
- [x] launch 파일 생성
- [x] **4륜 스키드스티어 특성 반영** (후진, 제자리 회전)
- [x] **robot_radius 기반 안전거리** (inflation 대신 lethal zone 사용)
- [x] **NavFn 플래너** (단순하고 예측 가능)
- [ ] **테스트 및 디버깅**
- [ ] 규칙 기능 확장 (금지구역 시각화 등)
- [ ] 다양한 맵에서 테스트
- [ ] **동적 환경 지원** (obstacle_layer 추가 + 빈번한 재계획)
- [ ] **적응형 robot_radius** (경로 실패 시 자동 축소 후 재시도)

### VER0 - drobot_simulation (✅ 완료, 유지보수)
- [x] 프론티어 탐색 구현
- [x] 모듈 분리 리팩토링
- [x] RViz 시각화 (경로, 목표 마커)
- [ ] 파라미터 정리 (VER1 참고하여 나중에)

---

## VER4 준비: Isaac Sim + PX4 (📋 계획)

### Phase 4.1: Isaac Sim 연동
- [ ] Isaac Sim 설치 (RTX 3070+ 필요)
- [ ] Drobot USD 모델 생성
- [ ] ROS2 브릿지 설정
- [ ] 센서 시뮬레이션 (LiDAR, Camera, IMU)
- [ ] 도메인 랜덤화 추가

### Phase 4.2: Isaac Lab (강화학습)
- [ ] Isaac Lab 환경 설정
- [ ] Observation/Action space 정의
- [ ] Reward function 설계
- [ ] Navigation policy 훈련
- [ ] 훈련된 모델 배포

### Phase 4.3: PX4 드론 통합
- [ ] PX4 SITL 설치
- [ ] MAVROS 브릿지 구성
- [ ] 이륙/착륙/호버링 구현
- [ ] 비행 제어 테스트

### Phase 4.4: 하이브리드 모드
- [ ] 지상/비행 모드 전환 로직
- [ ] 3D SLAM (RTAB-Map)
- [ ] 3D 경로 계획 (Octomap)

### Phase 4.5: AI 기반 네비게이션 (장기 목표)
- [ ] **AI Agent 기반 적응형 네비게이션**
  - 주변 환경 분석 (좁은 통로, 열린 공간 등)
  - 상황에 따라 robot_radius 자동 조절
  - 위험도 평가 및 경로 선택
- [ ] **적응형 Inflation (Adaptive Inflation)**
  - LiDAR로 좌우 여유 공간 실시간 측정
  - 넓은 공간: Local inflation 축소 (0.20m) → 빠른 이동
  - 좁은 통로: Local inflation 확대 (0.40m) → 안전 우선
  - Dynamic Reconfigure로 런타임 파라미터 변경
- [ ] 강화학습 기반 로컬 플래너 (Isaac Lab 연계)
- [ ] 엔드투엔드 네비게이션 (카메라 → 속도 직접 출력)

---

## Sim-to-Real Gap 개선 (나중에)

### 센서 노이즈
- [ ] LiDAR 가우시안 노이즈 (`stddev: 0.01`)
- [ ] IMU 드리프트 노이즈
- [ ] 오도메트리 covariance 설정

### 물리 모델
- [ ] 바퀴 마찰 계수 조정 (`mu: 0.8`, `mu2: 0.6`)
- [ ] 바퀴 슬립 추가 (`slip1/slip2: 0.002`)
- [ ] 로봇 관성(Inertia) 실제 무게 기반 계산

### 동적 환경
- [ ] 움직이는 장애물 월드 생성
- [ ] 사람 모델 추가 (actor)
- [ ] 동적 장애물 회피 테스트
- [ ] **Global costmap에 obstacle_layer 추가** (동적 환경용)
  - 현재: 정적 환경 최적화 (obstacle_layer 없음 → inflation 고정)
  - 동적 환경 시: obstacle_layer 추가 + update_frequency 높임

---

## 구조 리팩토링 (중요)

### 중복 파일 정리
- [ ] `nav2_params.yaml` 중복 제거 (simulation/ vs navigation/)
- [ ] `config.py` 중복 제거
- [ ] `navigation.launch.py` 중복 제거

### 패키지 역할 분리
- [ ] **drobot_description** 분리 (urdf/, meshes/ 만)
- [ ] **drobot_simulation** 정리 (worlds/, simulation.launch.py 만)
- [ ] **drobot_navigation** 통합 (auto_explorer, goal_navigator, nav2 설정)
- [ ] **drobot_bringup** 생성 (통합 런치 파일)

### 현재 문제점
```
drobot_simulation/     # 시뮬레이션 + 자동탐색 + Nav2 (너무 많음!)
drobot_navigation/     # 목표 네비 (simulation과 중복)
```

### 목표 구조
```
drobot_description/    # 로봇 모델만
drobot_simulation/     # 시뮬레이션만
drobot_navigation/     # 네비게이션 전체
drobot_bringup/        # 통합 런치
```

---

## 월드 생성 시스템

### 동적 맵 생성
- [ ] SDF 베이스 템플릿 (고정: physics, lighting, ground, 외벽)
- [ ] Python 동적 컨텐츠 생성기 (내부 벽, 가구, 장애물)
- [ ] YAML 설정 파일 → SDF 자동 생성
- [ ] ROS2 Spawn Service로 런타임 객체 추가/삭제
- [ ] Procedural 맵 생성 (maze, warehouse 등)

---

## 기타 개선사항

### 시각화
- [ ] STL 부위별 분리 (바퀴 회전 시각화)
- [ ] 금지구역 RViz 시각화
- [ ] 속도제한 구역 시각화

### 성능
- [ ] STL 폴리곤 최적화 (20MB → 5MB 목표)
- [ ] Costmap 업데이트 최적화

---

## 완료된 항목

### 2026-02-01
- [x] VER1 패키지 생성 (drobot_navigation)
- [x] 규칙 엔진 구현
- [x] CLAUDE.md, TODO.md 업데이트
- [x] 프론티어 탐색 한계 분석 → VER1 전환 결정
- [x] **이중 Inflation 전략 구현** (경로생성 vs 경로추종 분리)
- [x] **4륜 스키드스티어 파라미터** (후진 허용, 횡이동 불가)
- [x] **SmacPlannerHybrid 적용** (안전 경로 우선)
- [x] **네비게이션 전략 문서화** (CLAUDE.md)

### 2026-01-31
- [x] auto_explorer.py 모듈 분리 리팩토링
- [x] "Start occupied" 오류 해결
- [x] STL 원점 보정
- [x] RViz 목표/경로 시각화 추가
