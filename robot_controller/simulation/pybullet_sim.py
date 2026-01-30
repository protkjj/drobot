#!/usr/bin/env python3
"""
PyBullet 시뮬레이션 - ROS 없이 Python만으로 실행
키보드로 로봇 조종 가능

설치: pip install pybullet

조작법:
    W/S: 전진/후진
    A/D: 좌회전/우회전
    X: 정지
    Q: 종료
"""
import pybullet as p
import pybullet_data
import time
import os

# 설정
WHEEL_BASE = 0.24  # 바퀴 간 거리 (m)
WHEEL_RADIUS = 0.05  # 바퀴 반지름 (m)
MAX_VELOCITY = 10.0  # 최대 각속도 (rad/s)


class DrobotSimulator:
    def __init__(self):
        # PyBullet 초기화 (GUI 모드)
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 중력 설정
        p.setGravity(0, 0, -9.81)

        # 바닥 로드
        self.plane_id = p.loadURDF("plane.urdf")

        # 로봇 URDF 로드
        urdf_path = os.path.join(os.path.dirname(__file__), "../urdf/drobot.urdf")
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0.1],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )

        # 조인트 인덱스 찾기
        self.left_wheel_joint = None
        self.right_wheel_joint = None

        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name == 'left_wheel_joint':
                self.left_wheel_joint = i
            elif joint_name == 'right_wheel_joint':
                self.right_wheel_joint = i

        print(f"Left wheel joint: {self.left_wheel_joint}")
        print(f"Right wheel joint: {self.right_wheel_joint}")

        # 카메라 설정
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        # 현재 속도
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        print("\n=== Drobot PyBullet Simulator ===")
        print("조작법:")
        print("  W/S: 전진/후진")
        print("  A/D: 좌회전/우회전")
        print("  X: 정지")
        print("  Q: 종료")
        print("================================\n")

    def set_velocity(self, linear, angular):
        """cmd_vel과 동일한 방식으로 속도 설정"""
        self.linear_vel = linear
        self.angular_vel = angular

        # Differential drive 계산
        left_vel = (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        right_vel = (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS

        # 속도 제한
        left_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, left_vel))
        right_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, right_vel))

        # 모터 제어
        p.setJointMotorControl2(
            self.robot_id,
            self.left_wheel_joint,
            p.VELOCITY_CONTROL,
            targetVelocity=left_vel,
            force=10
        )
        p.setJointMotorControl2(
            self.robot_id,
            self.right_wheel_joint,
            p.VELOCITY_CONTROL,
            targetVelocity=right_vel,
            force=10
        )

    def get_robot_state(self):
        """로봇 상태 반환"""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        return {
            'position': pos,
            'orientation': euler,
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel
        }

    def run(self):
        """메인 루프"""
        linear_speed = 0.5  # m/s
        angular_speed = 2.0  # rad/s

        try:
            while True:
                # 키보드 입력 처리
                keys = p.getKeyboardEvents()

                linear = 0.0
                angular = 0.0

                # W: 전진
                if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
                    linear = linear_speed
                # S: 후진
                if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
                    linear = -linear_speed
                # A: 좌회전
                if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
                    angular = angular_speed
                # D: 우회전
                if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
                    angular = -angular_speed
                # X: 정지
                if ord('x') in keys and keys[ord('x')] & p.KEY_IS_DOWN:
                    linear = 0.0
                    angular = 0.0
                # Q: 종료
                if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
                    print("종료합니다...")
                    break

                # 속도 설정
                self.set_velocity(linear, angular)

                # 시뮬레이션 스텝
                p.stepSimulation()

                # 로봇 위치로 카메라 따라가기
                robot_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.5,
                    cameraYaw=45,
                    cameraPitch=-30,
                    cameraTargetPosition=robot_pos
                )

                time.sleep(1./240.)

        except KeyboardInterrupt:
            print("\n종료합니다...")
        finally:
            p.disconnect()


def main():
    sim = DrobotSimulator()
    sim.run()


if __name__ == '__main__':
    main()
