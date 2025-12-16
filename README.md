# mycobot320_moveit
## 🛠️ Update: MyCobot Gripper Control Fix (2025.12.16)

### 📝 Issue Description
- **문제점:** ROS 2 MoveIt/ROS2 Control에서 그리퍼 제어 명령(`FollowJointTrajectory`)을 보내도, 실제 하드웨어(Adaptive Gripper)가 반응하지 않는 문제 발생.
- **원인:** ROS 2 컨트롤러는 **Joint Position(각도)** 값을 보내지만, MyCobot 그리퍼는 별도의 **프로토콜 명령(0x66)**을 수신해야 동작함. 또한, 사용 중인 `mycobot_cpp` 라이브러리에 `write()` 또는 멤버 함수 `set_gripper_state()`가 부재함.

### ✅ Solution (Hardware Interface 수정)
`mycobot_hardware_interface` 패키지의 `write()` 함수 내에 **프로토콜 변환 로직**을 구현하여 해결함.

**주요 변경 사항:**
1.  **명령 변환 로직 추가:** `gripper_controller` 조인트의 입력값을 감지하여 그리퍼 상태(Open/Close)로 변환.
    - `cmd < -0.5` → **Close** (값: 1)
    - `cmd > -0.2` → **Open** (값: 0)
2.  **API 호출 방식 변경:**
    - 기존: `mycobot_->set_gripper_state()` (사용 불가)
    - 변경: `mycobot_->send(set_gripper_state(flag, speed))` (Command 객체 생성 후 전송)
3.  **통신 최적화:** `static` 변수를 사용하여 동일한 명령이 중복 전송되는 것을 방지 (Serial 통신 부하 감소).

### 🚀 How to Test (테스트 방법)
그리퍼가 정상 작동하는지 확인하기 위해 다음 명령어를 터미널에 입력합니다.

**1. 그리퍼 닫기 (Grip)**

ros2 topic pub --once /gripper_traj_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {frame_id: base_link},
  joint_names: ['gripper_controller'],
  points: [{positions: [-0.7], time_from_start: {sec: 1, nanosec: 0}}]}"

2. 그리퍼 열기 (Release)
ros2 topic pub --once /gripper_traj_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {frame_id: base_link},
  joint_names: ['gripper_controller'],
  points: [{positions: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}"
