#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_benchmark_global_planner.srv import ComputePathWithKPI
from geometry_msgs.msg import PoseStamped
import yaml, cv2, os
from ament_index_python.packages import get_package_share_directory

class BenchmarkClient(Node):
    def __init__(self):
        super().__init__('benchmark_client')
        self.path_client = self.create_client(ComputePathWithKPI, 'compute_path_kpi')
        self.map_param_client = self.create_client(SetParameters, '/map_server/set_parameters')
        self.planner_param_client = self.create_client(SetParameters, '/planner_server/set_parameters')
        self.map_lifecycle_client = self.create_client(ChangeState, '/map_server/change_state')

        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for path service...")
        base_path = get_package_share_directory('nav2_benchmark_global_planner')
        self.maps_dir = os.path.join(base_path, "maps")
        self.start_goal_sets = [
            ((5.0, 5.0), (6.0, 6.0)),
            ((5.0, 7.0), (7.0, 5.0)),
            ((6.0, 4.0), (4.0, 6.0)),
        ]
        self.planners = ["NavfnPlanner", "SmacPlanner2D", "SmacPlannerHybrid", "SmacPlannerLattice", "ThetaStarPlanner"]
        self.collision_models = [
            ("radius", [rclpy.parameter.Parameter("robot_radius", rclpy.Parameter.Type.DOUBLE, 0.2)]),
            ("footprint", [rclpy.parameter.Parameter("footprint", rclpy.Parameter.Type.DOUBLE_ARRAY,
                                                    [-0.2, -0.2, 0.2, -0.2, 0.2, 0.2, -0.2, 0.2])])
        ]

    def run_tests(self):
        maps = ['final_scenario.yaml']
        self.get_logger().info(f"Found {len(maps)} maps to test")
        
        for map_file in maps:
            map_path = os.path.join(self.maps_dir, map_file)
            self.get_logger().info(f"Testing map: {map_file}")
            self.reload_map(map_path)

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
                        self.get_logger().info(f"Testing path from {start} to {goal}")
                        self.call_path_service(img_color.copy(), map_yaml,
                                            map_file, planner_id, model_name, start, goal)

    def reload_map(self, yaml_file):
        pass
        # self.get_logger().info(f"Reloading map from {yaml_file}")
        # req = SetParameters.Request()
        # req.parameters = [rclpy.parameter.Parameter("yaml_filename", rclpy.Parameter.Type.STRING, yaml_file).to_parameter_msg()]
        # self.map_param_client.call_async(req)

        # self.get_logger().info("Cycling map server lifecycle")
        # change_req = ChangeState.Request()
        # change_req.transition.id = Transition.TRANSITION_DEACTIVATE
        # self.map_lifecycle_client.call_async(change_req)
        # change_req.transition.id = Transition.TRANSITION_ACTIVATE
        # self.map_lifecycle_client.call_async(change_req)

    def switch_planner(self, name, plugin):
        pass
        # self.get_logger().info(f"Switching planner to {name} ({plugin})")
        # req = SetParameters.Request()
        # req.parameters = [
        #     rclpy.parameter.Parameter("planner_plugins", rclpy.Parameter.Type.STRING_ARRAY, [name]).to_parameter_msg(),
        #     rclpy.parameter.Parameter(f"{name}.plugin", rclpy.Parameter.Type.STRING, plugin).to_parameter_msg()
        # ]
        # self.planner_param_client.call_async(req)

    def switch_collision_model(self, params):
        pass
        # param_names = [p.name for p in params]
        # self.get_logger().info(f"Switching collision model parameters: {param_names}")
        # req = SetParameters.Request()
        # req.parameters = [p.to_parameter_msg() for p in params]
        # self.planner_param_client.call_async(req)

    def call_path_service(self, img, map_yaml, map_file, planner_id, model_name, start, goal):
        self.get_logger().info(f"Requesting path: Map={map_file}, Planner={planner_id}, Model={model_name}, Start={start}, Goal={goal}")

        req = ComputePathWithKPI.Request()
        req.start = PoseStamped()
        req.start.header.frame_id = "map"
        req.start.pose.position.x, req.start.pose.position.y = start
        req.start.pose.orientation.w = 1.0

        req.goal = PoseStamped()
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x, req.goal.pose.position.y = goal
        req.goal.pose.orientation.w = 1.0
        
        # Additional required parameters
        req.use_start = True
        req.planner_id = planner_id
        self.get_logger().info("Calling path service...")
        future = self.path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()

        if result and result.success:
            self.get_logger().info(f"Path found! Length: {result.path_length:.2f}m, Planning time: {result.planning_time:.3f}s")
            self.draw_and_save(img, result, map_yaml, map_file, planner_id, model_name, start, goal)
        else:
            self.get_logger().error(f"Failed to compute path: {result.error if result else 'No response'}")

    def draw_and_save(self, img, result, map_yaml, map_file, planner_id, model_name, start, goal):
        resolution = map_yaml["resolution"]
        origin = map_yaml["origin"]

        for pose in result.path.poses:
            x = int((pose.pose.position.x - origin[0]) / resolution)
            y = int((pose.pose.position.y - origin[1]) / resolution)
            cv2.circle(img, (x, img.shape[0]-y), 2, (0,0,255), -1)

        text = f"{planner_id} | {model_name} | L={result.path_length:.2f} | T={result.planning_time:.3f}s"
        cv2.putText(img, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)

        start_str = f"{start[0]}_{start[1]}"
        goal_str = f"{goal[0]}_{goal[1]}"
        fname = f"results/{map_file[:-5]}_{planner_id}_{model_name}_S{start_str}_G{goal_str}.png"
        os.makedirs("results", exist_ok=True)
        cv2.imwrite(fname, img)
        self.get_logger().info(f"Saved: {fname}")

if(__name__ == '__main__'):
    rclpy.init()
    node = BenchmarkClient()
    node.run_tests()
    node.destroy_node()
    rclpy.shutdown()
