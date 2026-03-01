# TODO

## 패키지 구조 정리

- [ ] **drobot_description에서 시뮬레이션 에셋을 drobot_simulator로 분리**
  - `worlds/`, `models/` (88개), `object/`, `scripts/world_generator.py` → `drobot_simulator`로 이동
  - `drobot_description`에는 URDF, meshes만 남기기
  - `drobot_simulator`의 `CMakeLists.txt` 수정

- [ ] **drobot_scan_2d를 drobot_navigation으로 이름 변경**
  - 폴더명, `setup.py`, `setup.cfg`, `package.xml`, `resource/` 마커 파일 모두 변경
  - `drobot_bringup` 등 다른 패키지에서 참조하는 부분 수정

- [ ] **규칙 엔진을 drobot_strategy로 이동**
  - `drobot_scan_2d/rules/` → `drobot_strategy/drobot_strategy/rules/`
  - `drobot_bringup/config/navigation/rules.yaml` → `drobot_strategy/config/rules.yaml`
  - `goal_navigator.py`에서 import 경로 수정

- [ ] **drobot_bringup에서 UI 코드 분리**
  - `world_ui.py`를 별도 패키지(`drobot_ui`)로 분리하거나 구조 정리
  - `ui.launch.py` 참조 경로 수정

## 파일 정리

- [ ] **perception에서 학습용 데이터셋 파일 정리**
  - `perception/dataset/` (data.yaml, README, yolov8n.pt, yolo26n.pt)는 런타임에 불필요
  - 프로젝트 루트 `training/` 폴더로 이동하거나 별도 관리

## 메타데이터

- [ ] **package.xml에 누락된 설명과 라이선스 채우기**
  - 대상: perception, drobot_controller, drobot_scan_3d, drobot_strategy, drobot_simulator
  - `TODO: Package description` → 실제 설명 작성
  - `TODO: License declaration` → 라이선스 선택 (Apache-2.0, MIT 등)
