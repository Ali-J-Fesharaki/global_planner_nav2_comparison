#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_benchmark_global_planner.srv import ComputePathWithKPI
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import yaml, cv2, os
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
import time
import csv
class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)
        self.start_point = (3.0, 3.0)  # Default start point
        
    def broadcast_tf(self):
        """Broadcast the transform from map to base_link based on current start point"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Set the translation from the current start point
        t.transform.translation.x = self.start_point[0]
        t.transform.translation.y = self.start_point[1]
        t.transform.translation.z = 0.0
        
        # Set the rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
    def update_start_point(self, x, y):
        self.start_point = (x, y)
        self.get_logger().info(f"Updated TF start point to {self.start_point}")


class BenchmarkClient(Node):
    def __init__(self):
        super().__init__('benchmark_client')
        self.path_client = self.create_client(ComputePathWithKPI, 'compute_path_kpi')
        self.map_param_client = self.create_client(SetParameters, '/map_server/set_parameters')
        self.planner_param_client = self.create_client(SetParameters, '/planner_server/set_parameters')
        self.map_lifecycle_client = self.create_client(ChangeState, '/map_server/change_state')
        
        base_path = get_package_share_directory('nav2_benchmark_global_planner')
        self.maps_dir = os.path.join(base_path, "maps")
        self.start_goal_sets = [
            ((1.5, 1.5), (10.0, 10.0)),
            ((1.5, 1.5), (11.0, 1.5)),
            ((25.3, 1.5), (23.3, 47.5))
        ]
        self.planners = ["NavfnPlanner","NavfnPlanner_Astar","SmacPlanner2D", "SmacPlannerHybrid", "SmacPlannerLattice", "ThetaStarPlanner"]
        self.collision_models =["al"]
        
        # Store reference to TF publisher node to update start position
        self.tf_publisher = None
        self.services_available = False

    def set_tf_publisher(self, tf_publisher_node):
        self.tf_publisher = tf_publisher_node

    def check_services(self):
        """Check if all required services are available"""
        if (self.path_client.service_is_ready() and 
            self.map_param_client.service_is_ready() and
            self.planner_param_client.service_is_ready() and
            self.map_lifecycle_client.service_is_ready()):
            self.services_available = True
            return True
        return False

    def wait_for_services(self, timeout_sec=30.0):
        """Wait for all services to become available, with timeout"""
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if self.check_services():
                self.get_logger().info("All services are available!")
                return True
                
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(f"Timed out waiting for services after {timeout_sec} seconds")
                return False
                
            self.get_logger().info("Waiting for services to become available...")
            time.sleep(1.0)
        
        return False

    def run_tests(self):
        # Keep TF publishing regardless of service availability
        maps = [f for f in os.listdir(self.maps_dir) if f.endswith('.yaml')]
        #maps =["maze_orthogonal.yaml"]
        self.get_logger().info(f"Found {len(maps)} maps to test")
        
        # Wait for services, but continue with TF publishing even if they're not available
        if not self.wait_for_services():
            self.get_logger().error("Services not available. TF will continue to publish, but no benchmarks will run.")
            return
        
        for map_file in maps:
            map_path = os.path.join(self.maps_dir, map_file)
            self.get_logger().info(f"Testing map: {map_file}")
            self.reload_map(map_path)
            self.create_start_goal_sets(map_path)
            with open(map_path, "r") as f:
                map_yaml = yaml.safe_load(f)
            img = cv2.imread(os.path.join(self.maps_dir, map_yaml["image"]), cv2.IMREAD_GRAYSCALE)
            img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            for planner_id in self.planners:
                self.get_logger().info(f"Using planner: {planner_id}")
                for model_name, params in self.collision_models:
                    self.get_logger().info(f"Using collision model: {model_name}")
                    self.switch_collision_model(params)

                    for start, goal in self.start_goal_sets:
                        # Update start point in TF publisher node
                        # if self.tf_publisher:
                        #     self.tf_publisher.update_start_point(start[0], start[1])
                        
                        self.get_logger().info(f"Testing path from {start} to {goal}")
                        self.call_path_service(img_color.copy(), map_yaml,
                                            map_file, planner_id, model_name, start, goal)

    # Other methods remain unchanged...
    def reload_map(self, yaml_file):
        self.get_logger().info(f"Reloading map from {yaml_file}")
        time.sleep(2.0)  # Ensure map server is ready
        req = SetParameters.Request()
        req.parameters = [rclpy.parameter.Parameter("yaml_filename", rclpy.Parameter.Type.STRING, yaml_file).to_parameter_msg()]
        self.map_param_client.call_async(req)
        time.sleep(2.0)  # Ensure map server is ready

        self.get_logger().info("Cycling map server lifecycle")
        change_req = ChangeState.Request()
        change_req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.map_lifecycle_client.call_async(change_req)
        time.sleep(2.0)  # Ensure map server is ready
        change_req.transition.id = Transition.TRANSITION_ACTIVATE
        self.map_lifecycle_client.call_async(change_req)
        time.sleep(2.0)  # Ensure map server is ready


    def create_start_goal_sets(self, map_path):
        pass
    def switch_planner(self, name, plugin):
        pass
        # No changes to this method

    def switch_collision_model(self, params):
        pass
        # No changes to this method

    def call_path_service(self, img, map_yaml, map_file, planner_id, model_name, start, goal):
        if not self.services_available:
            self.get_logger().warn("Services not available. Skipping path computation.")
            return
            
        self.get_logger().info(f"Requesting path: Map={map_file}, Planner={planner_id}, Model={model_name}, Start={start}, Goal={goal}")

        req = ComputePathWithKPI.Request()
        req.start = PoseStamped()
        req.start.header.frame_id = "map"
        req.start.pose.position.x, req.start.pose.position.y = start
        print("start point: ")
        print(req.start.pose.position.x, req.start.pose.position.y)
        # Set orientation to a fixed quaternion representing 45 degrees rotation
        req.start.pose.orientation.x = 0.0
        req.start.pose.orientation.y = 0.0
        req.start.pose.orientation.z = 0.3826834323650898  # sin(45 degrees / 2)
        req.start.pose.orientation.w = 0.9238795325112867  # cos(45 degrees / 2)


        req.goal = PoseStamped()
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x, req.goal.pose.position.y = goal
        print("goal point: ")
        print(req.goal.pose.position.x, req.goal.pose.position.y)
        req.goal.pose.orientation.x = 0.0
        req.goal.pose.orientation.y = 0.0
        req.goal.pose.orientation.z = 0.3826834323650898  # sin(45 degrees / 2)
        req.goal.pose.orientation.w = 0.9238795325112867  # cos(45 degrees / 2)

        # Additional required parameters
        req.use_start = True
        req.planner_id = planner_id
        self.get_logger().info("Calling path service...")
        future = self.path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()

        if result and result.success:
            self.get_logger().info(f"Path found! Length: {result.path_length:.2f}m, Planning time: {result.planning_time:.3f}s")
            self.draw_and_save(img, result, map_yaml, map_file, planner_id, model_name, start, goal)
        else:
            self.get_logger().error(f"Failed to compute path from {start} to {goal}")

    def draw_and_save(self, img, result, map_yaml, map_file, planner_id, model_name, start, goal):
        resolution = map_yaml["resolution"]
        origin = map_yaml["origin"]

        for pose in result.path.poses:
            x = int((pose.pose.position.x - origin[0]) / resolution)
            y = int((pose.pose.position.y - origin[1]) / resolution)
            cv2.circle(img, (x, img.shape[0]-y), 2, (0,0,255), -1)

        # Prepare the CSV file path
        csv_file = "results/benchmark_results.csv"
        os.makedirs("results", exist_ok=True)

        # Check if the file exists to determine if headers need to be written
        file_exists = os.path.isfile(csv_file)

        # Write the results to the CSV file
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                # Write the header row if the file is new
                writer.writerow(["Map", "Planner", "Collision Model", "Path Length", "Planning Time", "Start", "Goal"])
            writer.writerow([
            map_file, 
            planner_id, 
            model_name, 
            f"{result.path_length:.2f}", 
            f"{result.planning_time:.3f}", 
            f"{start[0]},{start[1]}", 
            f"{goal[0]},{goal[1]}"
            ])

        start_str = f"{start[0]}_{start[1]}"
        goal_str = f"{goal[0]}_{goal[1]}"
        fname = f"results/{map_file[:-5]}_{planner_id}_{model_name}_S{start_str}_G{goal_str}.png"
        os.makedirs("results", exist_ok=True)
        cv2.imwrite(fname, img)
        self.get_logger().info(f"Saved: {fname}")


if(__name__ == '__main__'):
    rclpy.init()
    
    # Create the TF publisher node
    tf_publisher = TFPublisherNode()
    
    # Create benchmark client node
    benchmark_client = BenchmarkClient()
    
    # Connect the nodes
    benchmark_client.set_tf_publisher(tf_publisher)
    
    # Create multi-threaded executor to run both nodes
    executor = MultiThreadedExecutor(num_threads=1)
    #executor.add_node(tf_publisher)
    executor.add_node(benchmark_client)
    
    # Spin in a separate thread for TF publishing to continue regardless of service availability
    import threading

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Run the tests
    benchmark_client.run_tests()
    
    # Keep the program running to continue TF publishing even if benchmarks fail
    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    rclpy.shutdown()
    executor_thread.join()
