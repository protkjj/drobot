#!/usr/bin/env python3
"""
Pygame 2D 시뮬레이션 - ROS 없이 Python만으로 실행
Differential Drive 로봇 시뮬레이션

설치: pip install pygame

조작법:
    W/S: 전진/후진
    A/D: 좌회전/우회전
    X: 정지
    R: 리셋
    Q/ESC: 종료
"""
import pygame
import math
import sys

# 설정
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# 로봇 파라미터 (미터 단위, 화면에서는 픽셀로 변환)
WHEEL_BASE = 0.24  # 바퀴 간 거리 (m)
ROBOT_LENGTH = 0.3  # 로봇 길이 (m)
ROBOT_WIDTH = 0.2   # 로봇 너비 (m)
WHEEL_RADIUS = 0.05  # 바퀴 반지름 (m)

# 픽셀 변환 (1m = 100px)
SCALE = 100

# 색상
WHITE = (255, 255, 255)
BLACK = (30, 30, 30)
BLUE = (50, 100, 200)
RED = (200, 50, 50)
GREEN = (50, 200, 50)
GRAY = (100, 100, 100)
LIGHT_GRAY = (200, 200, 200)


class DrobotSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Drobot 2D Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        # 로봇 상태 (미터 단위)
        self.reset()

        # 속도 설정
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 2.0  # rad/s

        # 현재 입력
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # 궤적 기록
        self.trajectory = []

        print("\n=== Drobot 2D Simulator ===")
        print("조작법:")
        print("  W/S: 전진/후진")
        print("  A/D: 좌회전/우회전")
        print("  X: 정지")
        print("  R: 리셋")
        print("  Q/ESC: 종료")
        print("===========================\n")

    def reset(self):
        """로봇 상태 초기화"""
        self.x = SCREEN_WIDTH / 2 / SCALE  # 화면 중앙 (m)
        self.y = SCREEN_HEIGHT / 2 / SCALE
        self.theta = math.pi / 2  # 위쪽 방향 (rad)
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.trajectory = []

    def set_velocity(self, linear, angular):
        """cmd_vel 형식으로 속도 설정"""
        self.linear_vel = linear
        self.angular_vel = angular

        # Differential drive 계산
        self.left_wheel_vel = linear - (angular * WHEEL_BASE / 2.0)
        self.right_wheel_vel = linear + (angular * WHEEL_BASE / 2.0)

    def update(self, dt):
        """로봇 상태 업데이트 (Differential Drive Kinematics)"""
        # 속도 계산
        v = (self.left_wheel_vel + self.right_wheel_vel) / 2.0
        omega = (self.right_wheel_vel - self.left_wheel_vel) / WHEEL_BASE

        # 위치 업데이트
        self.x += v * math.cos(self.theta) * dt
        self.y -= v * math.sin(self.theta) * dt  # y축 반전 (화면 좌표계)
        self.theta += omega * dt

        # 각도 정규화
        self.theta = self.theta % (2 * math.pi)

        # 화면 경계 처리 (wrap around)
        if self.x < 0:
            self.x = SCREEN_WIDTH / SCALE
        elif self.x > SCREEN_WIDTH / SCALE:
            self.x = 0
        if self.y < 0:
            self.y = SCREEN_HEIGHT / SCALE
        elif self.y > SCREEN_HEIGHT / SCALE:
            self.y = 0

        # 궤적 기록
        screen_x = int(self.x * SCALE)
        screen_y = int(self.y * SCALE)
        if len(self.trajectory) == 0 or \
           (abs(self.trajectory[-1][0] - screen_x) > 2 or
            abs(self.trajectory[-1][1] - screen_y) > 2):
            self.trajectory.append((screen_x, screen_y))
            # 궤적 길이 제한
            if len(self.trajectory) > 500:
                self.trajectory.pop(0)

    def draw(self):
        """화면 그리기"""
        self.screen.fill(WHITE)

        # 그리드 그리기
        for x in range(0, SCREEN_WIDTH, 50):
            pygame.draw.line(self.screen, LIGHT_GRAY, (x, 0), (x, SCREEN_HEIGHT))
        for y in range(0, SCREEN_HEIGHT, 50):
            pygame.draw.line(self.screen, LIGHT_GRAY, (0, y), (SCREEN_WIDTH, y))

        # 궤적 그리기
        if len(self.trajectory) > 1:
            pygame.draw.lines(self.screen, GREEN, False, self.trajectory, 2)

        # 로봇 그리기
        self.draw_robot()

        # 정보 표시
        self.draw_info()

        pygame.display.flip()

    def draw_robot(self):
        """로봇 그리기"""
        # 화면 좌표로 변환
        cx = self.x * SCALE
        cy = self.y * SCALE

        # 로봇 크기 (픽셀)
        length = ROBOT_LENGTH * SCALE
        width = ROBOT_WIDTH * SCALE
        wheel_length = 0.08 * SCALE
        wheel_width = 0.04 * SCALE

        # 로봇 본체 꼭짓점 계산 (회전 적용)
        cos_t = math.cos(-self.theta + math.pi/2)
        sin_t = math.sin(-self.theta + math.pi/2)

        # 본체 사각형
        half_l = length / 2
        half_w = width / 2
        corners = [
            (-half_l, -half_w),
            (half_l, -half_w),
            (half_l, half_w),
            (-half_l, half_w)
        ]

        # 회전 변환
        rotated = []
        for x, y in corners:
            rx = x * cos_t - y * sin_t + cx
            ry = x * sin_t + y * cos_t + cy
            rotated.append((rx, ry))

        # 본체 그리기
        pygame.draw.polygon(self.screen, BLUE, rotated)
        pygame.draw.polygon(self.screen, BLACK, rotated, 2)

        # 전방 표시 (삼각형)
        front_x = cx + (half_l + 5) * cos_t
        front_y = cy + (half_l + 5) * sin_t
        pygame.draw.circle(self.screen, RED, (int(front_x), int(front_y)), 5)

        # 바퀴 그리기
        wheel_offset = WHEEL_BASE / 2 * SCALE
        for side in [-1, 1]:  # 왼쪽, 오른쪽
            # 바퀴 중심
            wx = cx + side * wheel_offset * (-sin_t)
            wy = cy + side * wheel_offset * cos_t

            # 바퀴 사각형
            wh_l = wheel_length / 2
            wh_w = wheel_width / 2
            wheel_corners = [
                (-wh_l, -wh_w),
                (wh_l, -wh_w),
                (wh_l, wh_w),
                (-wh_l, wh_w)
            ]

            wheel_rotated = []
            for x, y in wheel_corners:
                rx = x * cos_t - y * sin_t + wx
                ry = x * sin_t + y * cos_t + wy
                wheel_rotated.append((rx, ry))

            pygame.draw.polygon(self.screen, BLACK, wheel_rotated)

    def draw_info(self):
        """정보 패널 그리기"""
        info_lines = [
            f"Position: ({self.x:.2f}, {self.y:.2f}) m",
            f"Heading: {math.degrees(self.theta):.1f} deg",
            f"Linear vel: {self.linear_vel:.2f} m/s",
            f"Angular vel: {self.angular_vel:.2f} rad/s",
            f"Left wheel: {self.left_wheel_vel:.2f} m/s",
            f"Right wheel: {self.right_wheel_vel:.2f} m/s",
            "",
            "Controls: W/S/A/D, X=Stop, R=Reset, Q=Quit"
        ]

        # 배경
        pygame.draw.rect(self.screen, (240, 240, 240), (5, 5, 280, 180))
        pygame.draw.rect(self.screen, BLACK, (5, 5, 280, 180), 1)

        y = 10
        for line in info_lines:
            text = self.font.render(line, True, BLACK)
            self.screen.blit(text, (10, y))
            y += 20

    def handle_input(self):
        """키보드 입력 처리"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    return False
                if event.key == pygame.K_r:
                    self.reset()

        # 연속 키 입력
        keys = pygame.key.get_pressed()

        linear = 0.0
        angular = 0.0

        if keys[pygame.K_w]:
            linear = self.linear_speed
        if keys[pygame.K_s]:
            linear = -self.linear_speed
        if keys[pygame.K_a]:
            angular = self.angular_speed
        if keys[pygame.K_d]:
            angular = -self.angular_speed
        if keys[pygame.K_x]:
            linear = 0.0
            angular = 0.0

        self.set_velocity(linear, angular)
        return True

    def run(self):
        """메인 루프"""
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0  # 초 단위

            running = self.handle_input()
            self.update(dt)
            self.draw()

        pygame.quit()
        print("시뮬레이션 종료")


def main():
    sim = DrobotSimulator()
    sim.run()


if __name__ == '__main__':
    main()
