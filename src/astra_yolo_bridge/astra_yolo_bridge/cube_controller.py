#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

# --- [설정] ---
ARM_JOINT_NAMES = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
]
GRIPPER_JOINT_NAME = 'gripper_controller' 
GRIPPER_OPEN_VAL = 0.0
GRIPPER_CLOSE_VAL = -0.8

class CubeController(Node):
    def __init__(self):
        super().__init__('cube_controller')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_traj_controller/follow_joint_trajectory')

        self.get_logger().info("Waiting for Action Servers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server(timeout_sec=2.0)
        
        self.operate_gripper(open=True)
        self.state = 0 
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0
        
        # 1초마다 큐브 찾기
        self.timer = self.create_timer(1.0, self.control_loop)

    def control_loop(self):
        if self.state == 0:
            self.find_and_approach_cube()
        elif self.state == 2:
            self.move_to_home()

    def find_and_approach_cube(self):
        try:
            trans = self.tf_buffer.lookup_transform('base', 'cube_target', rclpy.time.Time())
        except Exception:
            self.get_logger().warn("🔍 Searching for cube...")
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z_target = trans.transform.translation.z
        
        # 1. 접근 위치 (Hover)
        z_hover = z_target + 0.20
        self.get_logger().info(f"📍 Found! Hovering at X={x:.2f}, Y={y:.2f}...")
        
        self.current_target_x = x
        self.current_target_y = y
        self.current_target_z = z_target

        # 상태 변경 (중복 실행 방지)
        self.state = 1 
        
        # [수정된 부분] 첫 이동은 제약을 풉니다 (strict_constraints=False)
        self.send_pose_goal(x, y, z_hover, strict_constraints=False, linear_motion=False, callback=self.on_hover_complete)

    def on_hover_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Hover Goal Rejected!")
            self.state = 0 # 다시 찾기
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.check_hover_result)

    def check_hover_result(self, future):
        result = future.result().result
        # [핵심] 성공했는지 확인! (에러 코드 1이 성공)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"❌ Hover Failed (Error: {result.error_code.val})")
            self.state = 0 # 실패하면 다시 찾기 모드로 복귀
            return

        # 성공했으면 하강 시작
        self.start_descent()

    def start_descent(self):
        z_grip = self.current_target_z + 0.18 # 높이 미세 조정
        self.get_logger().info("⬇️ Descending to Grip (Linear)...")
        
        # 직선 하강 (LIN) - 이때는 자세를 잡습니다 (strict=True)
        self.send_pose_goal(
            self.current_target_x, 
            self.current_target_y, 
            z_grip, 
            strict_constraints=True, 
            linear_motion=True,
            callback=self.on_down_complete
        )

    def on_down_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        goal_handle.get_result_async().add_done_callback(self.check_down_result)

    def check_down_result(self, future):
        if future.result().result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("❌ Descent Failed! Going Home...")
            self.move_to_home()
            return
            
        self.do_grip()

    def do_grip(self):
        self.get_logger().info("✊ Gripping...")
        self.operate_gripper(open=False)
        time.sleep(1.5) # 잡을 시간 충분히 주기
        
        z_lift = self.current_target_z + 0.25
        self.get_logger().info("⬆️ Lifting...")
        
        # 직선 상승
        self.send_pose_goal(
            self.current_target_x, 
            self.current_target_y, 
            z_lift, 
            strict_constraints=False, 
            linear_motion=True,
            callback=self.on_lift_complete
        )

    def on_lift_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        goal_handle.get_result_async().add_done_callback(lambda f: setattr(self, 'state', 2))

    def move_to_home(self):
        self.get_logger().info("🏠 Going Home...")
        self.state = 3 
        self.send_joint_goal([0.0]*6)

    def operate_gripper(self, open=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_OPEN_VAL if open else GRIPPER_CLOSE_VAL]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)
        self._gripper_client.send_goal_async(goal)

    def send_pose_goal(self, x, y, z, strict_constraints=True, linear_motion=False, callback=None):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.allowed_planning_time = 5.0
        
        # [중요] 파이프라인 명시
        if linear_motion:
            goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
            goal_msg.request.planner_id = "LIN"
            goal_msg.request.num_planning_attempts = 1
        else:
            goal_msg.request.pipeline_id = "ompl"
            goal_msg.request.planner_id = "RRTConnect"
            goal_msg.request.num_planning_attempts = 10

        # 제약 조건 설정
        pcm = PositionConstraint()
        pcm.header.frame_id = 'base'
        pcm.link_name = 'link6'
        pcm.weight = 1.0
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0
        pcm.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])]
        pcm.constraint_region.primitive_poses = [target_pose]

        ocm = OrientationConstraint()
        ocm.header.frame_id = 'base'
        ocm.link_name = 'link6'
        ocm.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        ocm.weight = 1.0
        
        if strict_constraints:
            ocm.absolute_x_axis_tolerance = 0.1
            ocm.absolute_y_axis_tolerance = 0.1
            ocm.absolute_z_axis_tolerance = 0.1 
        else:
            # 관대하게 풀어줌 (약 180도 허용)
            ocm.absolute_x_axis_tolerance = 3.14 
            ocm.absolute_y_axis_tolerance = 3.14
            ocm.absolute_z_axis_tolerance = 3.14

        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
        
        future = self._arm_client.send_goal_async(goal_msg)
        if callback:
            future.add_done_callback(callback)
        return future

    def send_joint_goal(self, joints):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTConnect"
        
        jc_list = []
        for i, val in enumerate(joints):
            jc = JointConstraint()
            jc.joint_name = ARM_JOINT_NAMES[i]
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            jc_list.append(jc)

        goal_msg.request.goal_constraints.append(Constraints(joint_constraints=jc_list))
        
        future = self._arm_client.send_goal_async(goal_msg)
        
        def home_done(f):
            if f.result().accepted:
                f.result().get_result_async().add_done_callback(self.on_home_complete)
        
        future.add_done_callback(home_done)

def main():
    rclpy.init()
    node = CubeController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()