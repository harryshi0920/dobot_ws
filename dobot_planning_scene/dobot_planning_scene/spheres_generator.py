import random

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def generate_spheres(
    radius,
    x,
    y,
    z,
    object_id,
):
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [radius]

    sphere_pose = Pose()
    sphere_pose.orientation.w = 1.0
    sphere_pose.position.x = x
    sphere_pose.position.y = y
    sphere_pose.position.z = z

    object = CollisionObject()
    object.header.frame_id = 'base_link'
    object.id = object_id
    object.primitives = [sphere]
    object.primitive_poses = [sphere_pose]
    object.operation = CollisionObject.ADD

    return object