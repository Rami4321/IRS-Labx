#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# --- Helper function to build a PoseStamped ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    """
    Create a PoseStamped (position + orientation) in the 'map' frame.

    Args:
        x (float): X coordinate in meters.
        y (float): Y coordinate in meters.
        yaw (float): Heading in radians (0 = along +X).

    Returns:
        PoseStamped: A goal pose for Nav2.
    """
    ps = PoseStamped()
    ps.header.frame_id = 'map'  # Nav2 goals are usually in 'map' frame
    ps.pose.position.x = x
    ps.pose.position.y = y

    # Convert yaw (in radians) into quaternion (Z-only)
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


def main():
    # 1) Initialise ROS 2 and create a node
    rclpy.init()
    node = rclpy.create_node('hs_waypoint_follower_nav2pose')

    # 2) Create an ActionClient for the NavigateToPose action
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # --- Function to send a goal and wait for result ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()

        # Update timestamp (required in headers)
        pose.header.stamp = node.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass  # Some nav2 versions may omit this field at times

        # Send the goal
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        if result is None:
            node.get_logger().error('No result returned.')
            return False

        node.get_logger().info('Goal reached successfully!')
        return True

    wp1 = make_pose(1.8,   3.0,   0.00250)
    wp2 = make_pose(5.34,  7.32,  0.00247)
    wp3 = make_pose(6.6,   13.0,  0.00256)
    wp4 = make_pose(25.0,  13.5,  0.00247)
    wp5 = make_pose(25.0,  0.114, 0.00247)
    wp6 = make_pose(6.72,  0.0423, -0.00143)

    waypoints = [wp1, wp2, wp3, wp4, wp5, wp6]

    # --- Patrol sequence ---
    for idx, wp in enumerate(waypoints, start=1):
        node.get_logger().info(f'Heading to waypoint {idx}...')
        success = send_and_wait(wp)
        if success:
            wait_seconds = 3.0  # pause after each waypoint
            node.get_logger().info(
                f'Waiting {wait_seconds:.0f} seconds at waypoint {idx}...'
            )
            time.sleep(wait_seconds)
        else:
            node.get_logger().warn(f'Failed to reach waypoint {idx}, skipping wait.')

            
    # 6) Shutdown node and ROS 2
    node.get_logger().info('Navigation sequence complete. Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
