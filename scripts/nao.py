#!/usr/bin/env python
import rospy
import math
from naoqi import ALProxy
from kinect_nao.msg import jointdatalist

LIMITS = {
    'HeadYaw': [-2.0, 2.0], 
    'HeadPitch': [-0.67, 0.51],
    'LShoulderPitch': [-2.0, 2.0], 
    'LShoulderRoll': [-0.31, 1.32],
    'RShoulderPitch': [-2.0, 2.0], 
    'RShoulderRoll': [-1.32, 0.31],
    'LElbowYaw': [-2.0, 2.0], 
    'LElbowRoll': [-1.54, -0.03],
    'RElbowYaw': [-2.0, 2.0],
    'RElbowRoll': [0.03, 1.54],
}

class NaoController:
    def __init__(self):
        rospy.init_node('nao_mykinect', anonymous=True)

        self.listener = rospy.Subscriber('/nao', jointdatalist, self.arm_control)
        ip = rospy.get_param('~ip', '192.168.31.162')
        port = int(rospy.get_param('~port', 9559))

        self.postureProxy = ALProxy("ALRobotPosture", ip, port)
        self.motionProxy = ALProxy("ALMotion", ip, port)

        for part in ["Head", "LArm", "RArm"]:
            self.motionProxy.setStiffnesses(part, 1.0)
        rospy.loginfo(self.motionProxy.getSummary())

    def arm_control(self, jdlist):
        angles = []
        parts = []
        speed = 1.0
        for jd in jdlist.joints:
            part = jd.Part.data
            angle = jd.Angles.data
            if angle < LIMITS[part][0] or angle > LIMITS[part][1]:
                error_msg = 'Wat? Limits man!'
                rospy.loginfo(error_msg)
            else:
                angles.append(angle)
                parts.append(part)
        self.motionProxy.setAngles(parts, angles, speed)

if __name__ == '__main__':
    try:
        NaoController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    