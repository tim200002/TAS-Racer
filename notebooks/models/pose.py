from .point import Point
from .quaternion import Quaternion
import geometry_msgs.msg as ros_geometry_msgs

class Pose:
    def __init__(self, coordinate: Point, quaternion: Quaternion):
        self.coordinate = coordinate
        self.quaternion = quaternion

    def __eq__(self, other):
        return self.coordinate == other.coordinate and self.angle == other.angle

    def __str__(self):
        return f"({self.coordinate.x}, {self.coordinate.y}, {self.quaternion.x}, {self.quaternion.y}, {self.quaternion.z}, {self.quaternion.w})"

    @classmethod
    def from_coordinate_and_yaw(cls, coordinate: Point, yaw: float):
        return Pose(coordinate=coordinate, quaternion=Quaternion.from_euler(0,0,yaw=yaw))

    def to_ros_message(self) -> ros_geometry_msgs.Pose:
        pose = ros_geometry_msgs.Pose()
        pose.position.x = self.coordinate.x
        pose.position.y = self.coordinate.y
        pose.position.z = 0.0
        pose.orientation = self.quaternion.to_ros_message()
        return pose
