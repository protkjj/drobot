#!/bin/bash
set -e

# GUI 권한 개방 (호스트 → 컨테이너)
xhost +local:docker
xhost +local:root

# Docker Compose 실행
docker compose up "$@"
