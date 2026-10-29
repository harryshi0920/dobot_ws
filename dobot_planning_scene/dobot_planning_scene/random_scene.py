import random
import rclpy

from dobot_planning_scene.spheres_generator import generate_spheres
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


def generate_random_spheres(num_spheres = 10, random_seed = None):
    random.seed(random_seed)

    spheres = []
    for i in range(num_spheres):
        radius = random.uniform(0.05, 0.2)
        x = random.uniform(-1.3, 1.3)
        y = random.uniform(-0.5, 0.5)
        z = random.uniform(0.1, 0.5)

        object_id = f'sphere_{i}'
        sphere_object = generate_spheres(radius, x, y, z, object_id)
        spheres.append(sphere_object)

    return spheres

def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = rclpy.create_node('random_scene')

    # Create a client for the PlanningScene service
    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    num_spheres = 10        # Number of random spheres to generate
    random_seed = None      # Seed for random number generator (None for random seed)

    # Generate random spheres
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.collision_objects = generate_random_spheres(num_spheres, random_seed)

    # Create a request for the PlanningScene service
    request = ApplyPlanningScene.Request()
    request.scene = scene

    # Wait for the service to be available
    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            node.destroy_node()
            return
        node.get_logger().info('Waiting for /apply_planning_scene...')


    future = client.call_async(request)
    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            response = future.result()
            if response.success:
                node.get_logger().info('Random spheres generate successfully')
            else:
                node.get_logger().error('Failed to generate random spheres')
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()