import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from nav_msgs.msg import Path
import time
import sys

# Status 5 es el código para 'SUCCEEDED' en action_msgs/msg/GoalStatus
GOAL_STATE_SUCCEEDED = 5

class SDVTranslate(Node):
    def __init__(self):
        super().__init__('sdv_translate')
        self.get_logger().info("SDVTranslate node started. Initializing clients...")

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 1. Cliente para el Planner Server (ComputePathToPose)
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        # 2. Cliente para el Controller Server (FollowPath)
        self.controller_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )
        
    def goal_callback(self, pose):
        self.get_logger().info("Received goal from /goal_pose. Starting planning process...")

        self.get_logger().info("Waiting for planner server to become available...")
        
        # Esperamos un máximo de 5 segundos
        if not self.planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Planner Server NOT available after 5 seconds. Aborting goal.")
            return

        self.get_logger().info("Planner Server is available. Sending goal...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map' 

        send_goal_future = self.planner_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.path_response)

    def path_response(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal de Planificación rechazado por el servidor.')
            return

        self.get_logger().info("Planner accepted the goal. Waiting for path result...")
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.follow_path_request)


    # ==============================================================
    # CORRECCIÓN: Verifica el estado de éxito (5) del PLANNER
    # ==============================================================
    def follow_path_request(self, future):
        
        # Verificamos que el estado final de la acción es SUCCEEDED (código 5)
        result_status = future.result().status
        
        if result_status != GOAL_STATE_SUCCEEDED:
            # Aquí capturaste el Status 4 (ABORTED/FAILED)
            self.get_logger().error(f"Planning failed! Planner Status: {result_status} (ABORTED/FAILED). Check planner_server logs!")
            return
            
        result = future.result().result
        path = result.path
        
        if not path.poses:
            self.get_logger().warn("Planner returned an EMPTY path. Cannot follow.")
            return

        self.get_logger().info(f"Planner returned a path with {len(path.poses)} points. Sending to Controller...")

        
        # ==============================================================
        # ENVÍO DEL GOAL AL CONTROLLER (Sigue igual)
        # ==============================================================
        if not self.controller_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Controller Server NOT available after 5s. Aborting navigation.")
            return

        controller_goal = FollowPath.Goal()
        controller_goal.path = path
        controller_goal.controller_id = 'FollowPath'

        send_goal_future = self.controller_client.send_goal_async(controller_goal)
        send_goal_future.add_done_callback(self.controller_finished)


    def controller_finished(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal de Seguimiento de Ruta rechazado por el controlador.')
            return

        self.get_logger().info("Controller accepted the path. Waiting for navigation result...")
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_finished)


    def navigation_finished(self, future):
        # Status 3 es 'GOAL_STATE_SUCCEEDED' en el resultado de FollowPath
        final_status = future.result().status
        if final_status == GOAL_STATE_SUCCEEDED:
            self.get_logger().info('✅ Navigation Finished: SUCCEEDED!')
        else:
            self.get_logger().warn(f'❌ Navigation Finished with non-success status: {final_status}')


def main(args=None):
    rclpy.init(args=args)
    node = SDVTranslate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()