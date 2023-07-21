import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Pose
 
import numpy as np # Scientific computing library for Python

prevPosX = 0.0
prevPosY = 0.0
prevPosZ = 0.0
prevOriX = 0.0
prevOriY = 0.0
prevOriZ = 0.0

 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  
  rollR = math.radians(roll)
  pitchR = math.radians(pitch)
  yawR = math.radians(yaw)
  
  q = [0] * 4
  
  q[1] = np.sin(rollR/2) * np.cos(pitchR/2) * np.cos(yawR/2) - np.cos(rollR/2) * np.sin(pitchR/2) * np.sin(yawR/2)
  q[2] = np.cos(rollR/2) * np.sin(pitchR/2) * np.cos(yawR/2) + np.sin(rollR/2) * np.cos(pitchR/2) * np.sin(yawR/2)
  q[3] = np.cos(rollR/2) * np.cos(pitchR/2) * np.sin(yawR/2) - np.sin(rollR/2) * np.sin(pitchR/2) * np.cos(yawR/2)
  q[0] = np.cos(rollR/2) * np.cos(pitchR/2) * np.cos(yawR/2) + np.sin(rollR/2) * np.sin(pitchR/2) * np.sin(yawR/2)
 
  return q

class TranslateQuaternion(Node):

    def __init__(self):
        super().__init__('translate_quaternion')
        self.subscription = self.create_subscription(
            Pose,
            'RPYinfo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.quat_info = self.create_publisher(Pose,'robocol/arm_desired_pose',10)

    def listener_callback(self, msg):
    
        global prevPosX, prevPosY, prevPosZ, prevOriX, prevOriY, prevOriZ
        #self.get_logger().info('I heard: "%s"' % msg.data)

        Roll = msg.orientation.y
        Pitch = msg.orientation.x
        Yaw = msg.orientation.z

        PosX = msg.position.x
        PosY = msg.position.y
        PosZ = msg.position.z

        if(Roll == prevOriX and Pitch == prevOriY and Yaw == prevOriZ and PosX == prevPosX and PosY == prevPosY and PosZ == prevPosZ):
            newPose = False
        else:
            newPose = True

        if(newPose):
        
            quat = get_quaternion_from_euler(Roll, Pitch, Yaw)
            
            posemsg = Pose()
            
            posemsg.position.x = PosX
            posemsg.position.y = PosY
            posemsg.position.z = PosZ
            posemsg.orientation.w = quat[0]
            posemsg.orientation.x = quat[1]
            posemsg.orientation.y = quat[2]
            posemsg.orientation.z = quat[3]
            	
            self.quat_info.publish(posemsg)

            prevOriX = Roll
            prevOriY = Pitch
            prevOriZ = Yaw

            prevPosX = PosX
            prevPosY = PosY
            prevPosZ = PosZ
        
        #print("Roll: " + str(Roll) + ", ")
        #print("Pitch: " + str(Pitch) + ", ")
        #print("Yaw: " + str(Yaw) + "\n")
        
        #print("W: " + str(quatmsg.w) + ", ")
        #print("X: " + str(quatmsg.x) + ", ")
        #print("Y: " + str(quatmsg.y) + ", ")
        #print("Z: " + str(quatmsg.z) + "\n")


def main(args=None):
    rclpy.init(args=args)

    translate_quaternion = TranslateQuaternion()

    rclpy.spin(translate_quaternion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    translate_quaternion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
