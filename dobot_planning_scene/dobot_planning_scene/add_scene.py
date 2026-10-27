import rclpy

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('add_box_node')

    # 1. 创建 /apply_planning_scene 客户端
    client = node.create_client(
        ApplyPlanningScene,
        '/apply_planning_scene'
    )

    node.get_logger().info(
        'Waiting for /apply_planning_scene...'
    )
    client.wait_for_service()

    # 2. 定义 box 的尺寸
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [
        0.5,   # X方向长度
        0.1,   # Y方向长度
        0.5    # Z方向长度
    ]

    # 3. 定义 box 的位置
    box_pose = Pose()
    box_pose.position.x = 0.5
    box_pose.position.y = 0.0
    box_pose.position.z = 0.25
    box_pose.orientation.w = 1.0

    # 4. 把形状和位置放进 CollisionObject
    collision_object = CollisionObject()
    collision_object.header.frame_id = 'base_link'
    collision_object.id = 'box1'
    collision_object.primitives = [box]
    collision_object.primitive_poses = [box_pose]
    collision_object.operation = CollisionObject.ADD

    # 5. 把障碍物放进 PlanningScene
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [collision_object]

    # 6. 创建服务请求
    request = ApplyPlanningScene.Request()
    request.scene = scene

    # 7. 发送请求
    future = client.call_async(request)

    rclpy.spin_until_future_complete(
        node,
        future
    )

    if future.result().success:
        node.get_logger().info('Box added successfully')
    else:
        node.get_logger().error('Failed to add box')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()