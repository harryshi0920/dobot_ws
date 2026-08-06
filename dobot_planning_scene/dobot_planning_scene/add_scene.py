import os

import rclpy
from rclpy.node import Node

import yaml
import trimesh

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


PACKAGE_NAME = 'dobot_planning_scene'


class PlanningSceneLoader(Node):

    def __init__(self):
        super().__init__('planning_scene_loader')

        # 默认读取安装目录中的 config/obstacles.yaml
        default_config_file = os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            'config',
            'obstacles.yaml'
        )

        self.declare_parameter('config_file', default_config_file)

        # 留空时使用 YAML 中的 frame_id
        self.declare_parameter('frame_id', '')

        self.config_file = self.get_parameter(
            'config_file'
        ).get_parameter_value().string_value

        self.frame_override = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value

        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

    def load_config(self):
        """读取 obstacles.yaml。"""

        if not os.path.isfile(self.config_file):
            raise FileNotFoundError(
                f'Obstacle configuration file not found: '
                f'{self.config_file}'
            )

        with open(self.config_file, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)

        if not isinstance(config, dict):
            raise ValueError(
                'The YAML root must be a dictionary.'
            )

        yaml_frame = config.get('frame_id', 'base_link')

        if self.frame_override.strip():
            frame_id = self.frame_override.strip()
        else:
            frame_id = str(yaml_frame)

        objects = config.get('objects', [])

        if not isinstance(objects, list):
            raise ValueError(
                'The "objects" field in obstacles.yaml must be a list.'
            )

        return frame_id, objects

    def make_pose(self, object_data):
        """根据 YAML 中的位置和四元数创建 Pose。"""

        position = object_data.get(
            'position',
            [0.0, 0.0, 0.0]
        )

        orientation = object_data.get(
            'orientation',
            [0.0, 0.0, 0.0, 1.0]
        )

        if len(position) != 3:
            raise ValueError(
                f'Object "{object_data.get("id")}" '
                f'position must contain 3 values.'
            )

        if len(orientation) != 4:
            raise ValueError(
                f'Object "{object_data.get("id")}" '
                f'orientation must contain 4 values [x, y, z, w].'
            )

        pose = Pose()

        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])

        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])

        return pose

    def make_box(self, object_data, frame_id):
        """创建 BOX 类型的 CollisionObject。"""

        object_id = object_data.get('id')

        if not object_id:
            raise ValueError('A box object is missing its "id".')

        size = object_data.get('size')

        if size is None or len(size) != 3:
            raise ValueError(
                f'Box "{object_id}" size must contain '
                f'3 values [x, y, z].'
            )

        if any(float(value) <= 0.0 for value in size):
            raise ValueError(
                f'Box "{object_id}" dimensions must be positive.'
            )

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            float(size[0]),
            float(size[1]),
            float(size[2])
        ]

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = str(object_id)

        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [
            self.make_pose(object_data)
        ]

        collision_object.operation = CollisionObject.ADD

        return collision_object

    def resolve_resource_path(self, resource):
        """将 package://、file:// 或相对路径转换为实际文件路径。"""

        if not resource:
            raise ValueError('Mesh object is missing "resource".')

        resource = str(resource)

        if resource.startswith('package://'):
            package_resource = resource[len('package://'):]

            if '/' not in package_resource:
                raise ValueError(
                    f'Invalid package resource URI: {resource}'
                )

            package_name, relative_path = package_resource.split(
                '/',
                1
            )

            package_share = get_package_share_directory(
                package_name
            )

            return os.path.join(
                package_share,
                relative_path
            )

        if resource.startswith('file://'):
            return resource[len('file://'):]

        if os.path.isabs(resource):
            return resource

        # 普通相对路径按照 obstacles.yaml 所在目录解析
        config_directory = os.path.dirname(self.config_file)

        return os.path.abspath(
            os.path.join(config_directory, resource)
        )

    def make_mesh(self, object_data, frame_id):
        """读取 STL 并创建 mesh 类型的 CollisionObject。"""

        object_id = object_data.get('id')

        if not object_id:
            raise ValueError('A mesh object is missing its "id".')

        mesh_path = self.resolve_resource_path(
            object_data.get('resource')
        )

        if not os.path.isfile(mesh_path):
            raise FileNotFoundError(
                f'Mesh file for "{object_id}" not found: '
                f'{mesh_path}'
            )

        scale = object_data.get(
            'scale',
            [1.0, 1.0, 1.0]
        )

        if len(scale) != 3:
            raise ValueError(
                f'Mesh "{object_id}" scale must contain '
                f'3 values [x, y, z].'
            )

        scale_x = float(scale[0])
        scale_y = float(scale[1])
        scale_z = float(scale[2])

        if scale_x <= 0 or scale_y <= 0 or scale_z <= 0:
            raise ValueError(
                f'Mesh "{object_id}" scale values must be positive.'
            )

        self.get_logger().info(
            f'Loading mesh "{object_id}": {mesh_path}'
        )

        loaded_mesh = trimesh.load_mesh(
            mesh_path,
            process=False
        )

        # 某些模型格式可能返回 Scene
        if isinstance(loaded_mesh, trimesh.Scene):
            loaded_mesh = loaded_mesh.dump(
                concatenate=True
            )

        if not isinstance(loaded_mesh, trimesh.Trimesh):
            raise ValueError(
                f'Unable to convert "{mesh_path}" into a triangle mesh.'
            )

        if loaded_mesh.is_empty:
            raise ValueError(
                f'Mesh "{object_id}" contains no geometry.'
            )

        mesh_message = Mesh()

        # STL 顶点转换为 geometry_msgs/Point
        for vertex in loaded_mesh.vertices:
            point = Point()

            point.x = float(vertex[0]) * scale_x
            point.y = float(vertex[1]) * scale_y
            point.z = float(vertex[2]) * scale_z

            mesh_message.vertices.append(point)

        # STL 三角面转换为 shape_msgs/MeshTriangle
        for face in loaded_mesh.faces:
            if len(face) != 3:
                raise ValueError(
                    f'Mesh "{object_id}" contains a non-triangle face.'
                )

            triangle = MeshTriangle()
            triangle.vertex_indices = [
                int(face[0]),
                int(face[1]),
                int(face[2])
            ]

            mesh_message.triangles.append(triangle)

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = str(object_id)

        collision_object.meshes = [mesh_message]
        collision_object.mesh_poses = [
            self.make_pose(object_data)
        ]

        collision_object.operation = CollisionObject.ADD

        self.get_logger().info(
            f'Mesh "{object_id}" loaded: '
            f'{len(mesh_message.vertices)} vertices, '
            f'{len(mesh_message.triangles)} triangles'
        )

        return collision_object

    def create_collision_object(self, object_data, frame_id):
        """根据 YAML 中的 type 选择对应创建函数。"""

        if not isinstance(object_data, dict):
            raise ValueError(
                'Every item in "objects" must be a dictionary.'
            )

        object_type = str(
            object_data.get('type', '')
        ).lower()

        if object_type == 'box':
            return self.make_box(
                object_data,
                frame_id
            )

        if object_type == 'mesh':
            return self.make_mesh(
                object_data,
                frame_id
            )

        raise ValueError(
            f'Unsupported object type "{object_type}" '
            f'for object "{object_data.get("id")}".'
        )

    def apply_scene(self):
        """读取配置并将全部物体发送给 MoveIt。"""

        frame_id, object_configs = self.load_config()

        collision_objects = []
        used_ids = set()

        for object_data in object_configs:
            object_id = object_data.get('id')

            if not object_id:
                raise ValueError(
                    'Every obstacle must have an "id".'
                )

            if object_id in used_ids:
                raise ValueError(
                    f'Duplicate obstacle ID: "{object_id}".'
                )

            used_ids.add(object_id)

            collision_object = self.create_collision_object(
                object_data,
                frame_id
            )

            collision_objects.append(collision_object)

        if not collision_objects:
            self.get_logger().warning(
                'No obstacles were found in the YAML file.'
            )
            return False

        self.get_logger().info(
            'Waiting for /apply_planning_scene service...'
        )

        while rclpy.ok():
            if self.client.wait_for_service(timeout_sec=1.0):
                break

            self.get_logger().info(
                'MoveGroup is not ready yet...'
            )

        if not rclpy.ok():
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects

        request = ApplyPlanningScene.Request()
        request.scene = scene

        self.get_logger().info(
            f'Applying {len(collision_objects)} collision objects '
            f'in frame "{frame_id}"...'
        )

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response is not None and response.success:
            self.get_logger().info(
                'Planning scene applied successfully.'
            )

            for collision_object in collision_objects:
                self.get_logger().info(
                    f'Added obstacle: {collision_object.id}'
                )

            return True

        self.get_logger().error(
            'MoveIt failed to apply the planning scene.'
        )

        return False


def main(args=None):
    """ROS 2 console_scripts 的程序入口。"""

    rclpy.init(args=args)

    node = None

    try:
        node = PlanningSceneLoader()
        node.apply_scene()

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f'Failed to load planning scene: {error}'
            )
        else:
            print(f'Failed to start planning scene loader: {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()