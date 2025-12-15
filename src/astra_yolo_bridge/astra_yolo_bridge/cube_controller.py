#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
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
GRIPPER_CLOSE_VAL = -0.7 

class CubeController(Node):
    def __init__(self):
        super().__init__('cube_controller')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_traj_controller/follow_joint_trajectory')

        self.get_logger().info("Waiting for Action Servers...")
        self._arm_client.wait_for_server()
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Gripper server not found!")
        else:
            self.get_logger().info("✅ Gripper Server Connected!")

        self.operate_gripper(open=True)
        self.state = 0 
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0
        
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
        
        z_hover = z_target + 0.20
        self.get_logger().info(f"📍 Found! Hovering at X={x:.2f}, Y={y:.2f}...")
        
        self.current_target_x = x
        self.current_target_y = y
        self.current_target_z = z_target

        if self.send_pose_goal(x, y, z_hover):
            self.state = 1 

    # [수정 1] future 인자 추가!
    def on_hover_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        
        # Hover 완료 후 결과 대기 -> 성공하면 하강
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.start_descent)

    def start_descent(self, future):
        # 실제 하강 시작
        z_grip = self.current_target_z + 0.17 
        self.get_logger().info("⬇️ Descending to Grip...")
        
        future_down = self.send_pose_goal(self.current_target_x, self.current_target_y, z_grip, wait=False)
        future_down.add_done_callback(self.on_down_complete)

    def on_down_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.do_grip)

    # [수정 2] future 인자 추가!
    def do_grip(self, future):
        self.get_logger().info("✊ Gripping...")
        self.operate_gripper(open=False)
        time.sleep(1.0)
        
        z_lift = self.current_target_z + 0.25
        self.get_logger().info("⬆️ Lifting...")
        
        lift_future = self.send_pose_goal(self.current_target_x, self.current_target_y, z_lift, wait=False)
        lift_future.add_done_callback(self.on_lift_complete)

    def on_lift_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        res_future = goal_handle.get_result_async()
        
        def finish_lift(f):
            self.state = 2 # 홈으로 이동
        res_future.add_done_callback(finish_lift)

    def move_to_home(self):
        self.get_logger().info("🏠 Going Home...")
        self.state = 3 
        if self.send_joint_goal([0.0]*6):
            pass

    def on_home_complete(self, future):
        self.get_logger().info("👐 Releasing Cube...")
        self.operate_gripper(open=True)
        time.sleep(2.0)
        self.state = 0 

    def operate_gripper(self, open=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_OPEN_VAL if open else GRIPPER_CLOSE_VAL]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)
        self._gripper_client.send_goal_async(goal)

    def send_pose_goal(self, x, y, z, wait=True):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

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
        ocm.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 3.14
        ocm.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
        
        future = self._arm_client.send_goal_async(goal_msg)
        if wait:
            future.add_done_callback(self.on_hover_complete)
        return future

    def send_joint_goal(self, joints):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

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
        future.add_done_callback(self.home_done_cb)
        return True

    def home_done_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = 2 
            return
        res = goal_handle.get_result_async()
        res.add_done_callback(self.on_home_complete)

def main():
    rclpy.init()
    node = CubeController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()