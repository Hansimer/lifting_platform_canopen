#!/usr/bin/env python3
"""
moveit_bridge.py

MoveIt2 规划 + 直接发送轨迹到控制器
绕过 MoveIt2 轨迹执行器的时间戳问题

流程:
  1. 调用 MoveIt2 move_action (plan_only=True) 获取规划轨迹
  2. 修复轨迹时间戳（确保严格递增）
  3. 直接发送给 lifting_platform_controller

用法:
  ros2 run lifting_platform_canopen moveit_bridge.py --ros-args -p target:=0.3
  ros2 run lifting_platform_canopen moveit_bridge.py --ros-args -p target:=0.8 -p duration:=8.0
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration


class MoveItBridge(Node):

    def __init__(self):
        super().__init__("moveit_bridge")

        # ── 参数 ──
        self.declare_parameter("target", 0.5)
        self.declare_parameter("duration", 300.0)
        self.declare_parameter("group", "lifting_platform")
        self.declare_parameter("joint", "updown")

        self.target = self.get_parameter("target").value
        self.duration = self.get_parameter("duration").value
        self.group = self.get_parameter("group").value
        self.joint = self.get_parameter("joint").value

        # ── Action 客户端 ──
        # MoveIt2 规划
        self._plan_client = ActionClient(self, MoveGroup, "move_action")
        # 控制器执行
        # JTC 运行在 /lifting_platform 命名空间下，完整 action 路径为：
        #   /lifting_platform/lifting_platform_controller/follow_joint_trajectory
        self._exec_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/lifting_platform/lifting_platform_controller/follow_joint_trajectory",
        )

        self.get_logger().info(f"目标: {self.joint} = {self.target:.3f}m")
        self.get_logger().info(f"期望执行时间: {self.duration:.1f}s")

        # 等待服务就绪
        self.get_logger().info("等待 MoveIt2 move_action server...")
        if not self._plan_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("MoveIt2 move_action server 未找到!")
            rclpy.shutdown()
            return

        self.get_logger().info("等待 lifting_platform_controller action server...")
        if not self._exec_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("控制器 action server 未找到!")
            rclpy.shutdown()
            return

        self.get_logger().info("服务就绪，开始规划...")
        self._send_plan_request()

    # ================================================================
    #  第一步: MoveIt2 规划
    # ================================================================

    def _send_plan_request(self):
        """发送规划请求 (plan_only=True, 不执行)"""
        goal = MoveGroup.Goal()

        # 规划请求
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 500.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        # 目标约束
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.joint
        jc.position = self.target
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.request = req
        goal.planning_options.plan_only = True   # ★ 只规划不执行
        goal.planning_options.replan = False

        future = self._plan_client.send_goal_async(goal)
        future.add_done_callback(self._plan_goal_cb)

    def _plan_goal_cb(self, future):
        """规划请求被接受/拒绝"""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("规划请求被 move_group 拒绝!")
            rclpy.shutdown()
            return

        self.get_logger().info("规划请求已接受，等待结果...")
        handle.get_result_async().add_done_callback(self._plan_result_cb)

    def _plan_result_cb(self, future):
        """规划完成，获取轨迹并修复时间戳"""
        result = future.result().result

        # 检查规划是否成功
        if result.error_code.val != 1:   # moveit_msgs MoveItErrorCodes::SUCCESS = 1
            self.get_logger().error(f"规划失败! error_code = {result.error_code.val}")
            rclpy.shutdown()
            return

        # 获取规划好的轨迹
        trajectory = result.planned_trajectory.joint_trajectory
        num_points = len(trajectory.points)

        self.get_logger().info(
            f"规划成功! {num_points} 个路径点, "
            f"关节: {trajectory.joint_names}"
        )

        # ★ 修复时间戳
        self._fix_timestamps(trajectory)

        # 打印轨迹摘要
        self._log_trajectory(trajectory)

        # 发送给控制器执行
        self._send_execute_goal(trajectory)

    # ================================================================
    #  第二步: 修复时间戳
    # ================================================================

    def _fix_timestamps(self, trajectory):
        """
        修复轨迹时间戳，确保:
        1. 第一个点有合理的时间 (> 0)
        2. 所有点时间严格递增
        3. 总时间约等于 self.duration
        """
        num_points = len(trajectory.points)
        if num_points == 0:
            return

        # 如果所有时间戳都是 0 或无效，重新分配
        all_zero = all(
            (p.time_from_start.sec == 0 and p.time_from_start.nanosec == 0)
            for p in trajectory.points
        )

        if all_zero:
            self.get_logger().warn("轨迹时间戳全为 0，重新分配...")
            for i, point in enumerate(trajectory.points):
                # 均匀分配时间: 从 0.1s 到 self.duration
                t = 0.1 + (self.duration - 0.1) * i / max(num_points - 1, 1)
                sec = int(t)
                nsec = int((t - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nsec)
        else:
            # 时间戳不全为 0，但可能不严格递增，修复之
            self.get_logger().info("检查时间戳严格递增性...")
            for i in range(num_points):
                if i == 0:
                    # 第一个点确保至少 0.01s
                    t_ns = (trajectory.points[i].time_from_start.sec * 1_000_000_000
                            + trajectory.points[i].time_from_start.nanosec)
                    if t_ns < 10_000_000:   # < 10ms
                        trajectory.points[i].time_from_start = Duration(
                            sec=0, nanosec=10_000_000
                        )
                else:
                    prev = trajectory.points[i - 1].time_from_start
                    prev_ns = prev.sec * 1_000_000_000 + prev.nanosec

                    curr = trajectory.points[i].time_from_start
                    curr_ns = curr.sec * 1_000_000_000 + curr.nanosec

                    if curr_ns <= prev_ns:
                        # 强制比前一个点大 10ms
                        new_ns = prev_ns + 10_000_000
                        trajectory.points[i].time_from_start = Duration(
                            sec=int(new_ns // 1_000_000_000),
                            nanosec=int(new_ns % 1_000_000_000),
                        )

    # ================================================================
    #  第三步: 发送给控制器执行
    # ================================================================

    def _send_execute_goal(self, trajectory):
        """直接发送轨迹给 lifting_platform_controller"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("发送轨迹到 lifting_platform_controller...")

        future = self._exec_client.send_goal_async(
            goal, feedback_callback=self._exec_feedback_cb
        )
        future.add_done_callback(self._exec_goal_cb)

    def _exec_feedback_cb(self, feedback_msg):
        """执行过程中的反馈"""
        fb = feedback_msg.feedback
        if fb.actual.positions:
            desired = fb.desired.positions[0] if fb.desired.positions else 0.0
            actual = fb.actual.positions[0]
            self.get_logger().info(
                f"目标: {desired:.4f}  实际: {actual:.4f}"
            )

    def _exec_goal_cb(self, future):
        """控制器接受/拒绝轨迹"""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("轨迹被控制器拒绝!")
            self.get_logger().error("请检查 canopen_bringup 是否正常运行")
            rclpy.shutdown()
            return

        self.get_logger().info("轨迹已接受，执行中...")
        handle.get_result_async().add_done_callback(self._exec_result_cb)

    def _exec_result_cb(self, future):
        """执行完成"""
        result = future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("到达目标!")
        elif result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            self.get_logger().warn("到达但超出容差")
        elif result.error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            self.get_logger().warn("路径跟踪超出容差")
        else:
            self.get_logger().error(f"执行失败, error_code = {result.error_code}")

        rclpy.shutdown()

    # ================================================================
    #  工具函数
    # ================================================================

    def _log_trajectory(self, trajectory):
        """打印轨迹摘要"""
        num_points = len(trajectory.points)
        self.get_logger().info(f"轨迹点数: {num_points}")
        self.get_logger().info(f"关节: {trajectory.joint_names}")

        # 打印首尾点
        first = trajectory.points[0]
        last = trajectory.points[-1]
        t_first = first.time_from_start.sec + first.time_from_start.nanosec / 1e9
        t_last = last.time_from_start.sec + last.time_from_start.nanosec / 1e9

        self.get_logger().info(
            f"起点: pos={first.positions[0]:.4f}, t={t_first:.3f}s"
        )
        self.get_logger().info(
            f"终点: pos={last.positions[0]:.4f}, t={t_last:.3f}s"
        )

        # 打印所有点 (如果不多)
        if num_points <= 10:
            for i, pt in enumerate(trajectory.points):
                t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
                self.get_logger().info(
                    f"  [{i}] pos={pt.positions[0]:.4f}, t={t:.3f}s"
                )


def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    rclpy.spin(node)


if __name__ == "__main__":
    main()