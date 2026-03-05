# Docker 설치 가이드

Gazebo/RViz 등 GUI가 필요하므로 **Ubuntu에서 사용**을 권장합니다.

## 1. 기존 Docker 완전 제거

```bash
sudo apt-get remove docker docker-engine docker.io containerd runc docker-compose
sudo apt-get autoremove -y
```

## 2. Docker 엔진 및 최신 Compose 설치

```bash
sudo apt-get update
sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common

# Docker 공식 GPG 키 추가
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# apt 저장소 등록
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

## 3. 권한 설정

```bash
sudo usermod -aG docker $USER
newgrp docker
```

## 4. GUI 설정 (ROS 2 Jazzy/Gazebo용)

로그인 시 자동으로 Docker GUI 권한을 열도록 설정합니다.

```bash
xhost +local:docker && xhost +local:root

mkdir -p ~/.config/autostart
cat > ~/.config/autostart/xhost-docker.desktop << EOF
[Desktop Entry]
Type=Application
Name=xhost Docker
Exec=bash -c "xhost +local:docker && xhost +local:root"
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
EOF
```

첫 줄로 즉시 권한이 열리고, autostart 등록으로 이후 로그인 시에도 자동 적용됩니다.

## 5. 설치 확인

```bash
docker --version
docker compose version
docker run hello-world
```

모두 정상 출력되면 Docker 설치 완료입니다. 프로젝트 빌드 및 실행 방법은 별도 안내됩니다.
