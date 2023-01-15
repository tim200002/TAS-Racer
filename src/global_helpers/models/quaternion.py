import math 
from math import sin, cos, atan2, asin
import geometry_msgs.msg as ros_geometry_msgs

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __eq__(self, other):
        return self.x == other.y and self.y == other.z and self.z == other.x and self.w == other.w

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z}, {self.w})"

    def conjugate(self):
        return Quaternion(self.x, self.y, self.z, -self.w)
    
    def norm(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    def inv(self):
        conjugated = self.conjugate()
        return Quaternion.scalar_multiply(1/conjugated.norm(), conjugated)

    @classmethod
    def scalar_multiply(cls, alpha, Q):
        return Quaternion(alpha*Q.x, alpha*Q.y, alpha*Q.z, alpha*Q.w)
    
    @classmethod
    def multiply(cls, Q1, Q2):
        x_new = Q1.w * Q2.x + Q1.x * Q2.w + Q1.y * Q2.z - Q1.z *Q2.y
        y_new = Q1.w * Q2.y - Q1.x * Q2.z + Q1.y * Q2.w + Q1.z *Q2.y
        z_new = Q1.w * Q2.z + Q1.x * Q2.y - Q1.y * Q2.x + Q1.z *Q2.w
        w_new = Q1.w * Q2.w - Q1.x * Q2.x - Q1.y * Q2.y - Q1.z *Q2.z

        return Quaternion(x_new, y_new, z_new, w_new)
    
    @classmethod 
    def from_euler(cls, roll, pitch, yaw):
        x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
            cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
            sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * sin(pitch/2) * sin(yaw/2)       

        return Quaternion(x,y,z,w)
    
    def to_euler(self):
        yaw = atan2(2.0*(self.y*self.z + self.w*self.x), self.w*self.w - self.x*self.x - self.y*self.y + self.z*self.z)
        pitch = asin(-2.0*(self.x*self.z - self.w*self.y))
        roll = atan2(2.0*(self.x*self.y + self.w*self.z), self.w*self.w + self.x*self.x - self.y*self.y - self.z*self.z)

        return roll, pitch, yaw

    def to_ros_message(self) -> ros_geometry_msgs.Quaternion:
        quaternion = ros_geometry_msgs.Quaternion()
        quaternion.x = self.x
        quaternion.y = self.y
        quaternion.z = self.z
        quaternion.w = self.w
        return quaternion
    
    @classmethod
    def from_ros_message(cls, Q: ros_geometry_msgs.Quaternion):
        return Quaternion(Q.x, Q.y, Q.z, Q.w)




