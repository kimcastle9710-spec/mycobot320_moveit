📌 Overview

이 프로젝트는 여러 ROS2 오픈소스(MoveIt2, ros2_control, myCobot SDK) 를 기반으로
실제 myCobot320 로봇을 구동하기 위해 시스템 통합(System Integration) 에 초점을 맞춘 프로젝트입니다.

단순히 오픈소스를 사용하는 수준을 넘어,
실제 하드웨어 환경에서 발생한 인터페이스 불일치, 제어 불가, 통신 과부하 문제를 분석하고 해결하는 것을 목표로 했습니다.

📂 Repository Structure
src/
 ├─ mycobot/
 │   ├─ mycobot_hardware_interface/   # Custom ROS2 Control Interface
 │   └─ mycobot/                      # myCobot SDK 기반 제어 코드
 ├─ adaptive_gripper_config/          # MoveIt2 gripper configuration
 ├─ ros2_astra_camera/                # Astra camera ROS2 node
 ├─ astra_yolo_bridge/                # Vision integration
 └─ astra_capture.py                  # Camera capture utility

🎯 What I Did (My Contribution)

본 프로젝트에서 제가 집중한 역할은 다음과 같습니다.
🔧 MoveIt2 ↔ ros2_control ↔ myCobot 하드웨어 통합

🔁 ROS2 Control Hardware Interface 직접 수정

✋ 그리퍼 제어 불가 문제 원인 분석 및 해결

⚙ 실제 로봇 구동 중 발생하는 jitter/중복 명령 문제 개선

🧪 실물 로봇 기반 반복 테스트 및 안정화

이 프로젝트는 “새 알고리즘 구현”이 아니라
“이미 존재하는 오픈소스들이 실제 로봇에서 제대로 동작하도록 만드는 과정” 에 가깝습니다.

🛠 Tech Stack

ROS2 (Humble)
MoveIt2
ros2_control
C++ / Python
myCobot320 (Real Hardware)
Serial Communication (Vendor SDK)

🧩 System Architecture
MoveIt2 (Planning)
        ↓
ros2_control (JointTrajectory)
        ↓
Custom Hardware Interface  ← [Modified]
        ↓
myCobot SDK (Serial Protocol)
        ↓
myCobot320 (Real Robot)

MoveIt2는 표준 FollowJointTrajectory 메시지를 생성
하지만 myCobot 그리퍼는 해당 인터페이스로 제어 불가
Hardware Interface 레벨에서 프로토콜 변환 로직을 직접 구현
🚨 Problem & Solution (Core of This Project)
❌ Problem

로봇 팔 조인트는 정상 동작
그리퍼는 MoveIt2 명령으로 전혀 동작하지 않음
원인:
myCobot 그리퍼는 전용 프로토콜 명령(0x66) 이 필요
기존 ROS2 드라이버에는 해당 기능이 구현되어 있지 않음

✅ Solution
mycobot_hardware_interface의 write() 함수를 직접 수정하여 해결
1️⃣ Gripper Command Translation
MoveIt2에서 전달되는 gripper_controller 조인트 값을 감지
값에 따라 그리퍼 상태로 변환
cmd < -0.5  → Close  (flag = 1)
cmd > -0.2  → Open   (flag = 0)
set_gripper_state(flag, speed) 프로토콜 명령 생성 후 전송
2️⃣ Duplicate Command Filtering
동일한 그리퍼 명령이 반복 전송되지 않도록 static 변수 사용
Serial 통신 부하 및 jitter 감소
3️⃣ Arm Command Optimization
이전 명령과 현재 명령의 변화량을 비교
변화가 미미하면 명령 전송 생략
실제 로봇에서 발생하던 미세 떨림 현상 개선

🧪 How to Test (Gripper Example)
Gripper Close
ros2 topic pub --once /gripper_traj_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['gripper_controller'],
  points: [{positions: [-0.7], time_from_start: {sec: 1}}]
}"

Gripper Open
ros2 topic pub --once /gripper_traj_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['gripper_controller'],
  points: [{positions: [0.0], time_from_start: {sec: 1}}]
}"

