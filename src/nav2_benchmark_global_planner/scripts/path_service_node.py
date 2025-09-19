#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav2_benchmark_global_planner.srv import ComputePathWithKPI
import psutil, time

class PathServiceNode(Node):
    def __init__(self):
        super().__init__('path_service_node')
        self.get_logger().info('Initializing path service node')
        self.srv = self.create_service(ComputePathWithKPI, 'compute_path_kpi', self.handle_request)
        self.get_logger().info('Service "compute_path_kpi" created')
        
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.get_logger().info('Action client for "compute_path_to_pose" created')
        
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for planner action server...")
        self.get_logger().info("Planner action server is available")

    def handle_request(self, request, response):
        self.get_logger().info(f"Received path planning request from ({request.start.pose.position.x:.2f}, {request.start.pose.position.y:.2f}) to ({request.goal.pose.position.x:.2f}, {request.goal.pose.position.y:.2f})")
        
        action_goal = ComputePathToPose.Goal()
        action_goal.start = request.start
        action_goal.goal = request.goal
        action_goal.planner_id = request.planner_id
        action_goal.use_start = request.use_start
        cpu_before = psutil.cpu_percent(interval=None)
        start_time = time.time()

        # Send the goal and wait for the result
        self.get_logger().info("Sending goal to compute_path_to_pose action server")
        send_goal_future = self.action_client.send_goal_async(action_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.result():
            self.get_logger().error("Failed to send goal to action server")
            response.success = False
            return response
            
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by action server")
            response.success = False
            return response
        
        self.get_logger().info("Goal accepted by action server, waiting for result")    
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        elapsed = time.time() - start_time
        cpu_after = psutil.cpu_percent(interval=None)
        
        result = result_future.result().result
        
        if result and result.path.poses:
            path_length = self.compute_path_length(result.path)
            self.get_logger().info(f"Path planning successful: length={path_length:.2f}m, time={elapsed:.3f}s, CPU delta={cpu_after - cpu_before:.1f}%")
            
            response.path = result.path
            response.path_length = path_length
            response.planning_time = elapsed
            response.cpu_usage = cpu_after - cpu_before
            response.success = True
        else:
            self.get_logger().error("Path planning failed: No valid path returned")
            response.success = False

        return response

    def compute_path_length(self, path):
        length = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            length += (dx**2 + dy**2)**0.5
        return length

if(__name__ == '__main__'):
    print("Starting Path Service Node...")
    rclpy.init()
    node = PathServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
