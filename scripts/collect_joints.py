#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np

from kinect_nao.msg import jointdata, jointdatalist
from std_msgs.msg import Float32, String

FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]

class Tracker():

    def __init__(self):
        rospy.init_node('tracker_mykinect', anonymous=True)
    	self.myFrame = '/nao'
    	self.pub_trans = rospy.Publisher(self.myFrame, jointdatalist, queue_size=10)
        rate = rospy.get_param('~rate', 3)

        r = rospy.Rate(rate)

        self.tf = tf.TransformListener()
        rospy.loginfo("Start tracking for 5s...")
        rospy.sleep(5.0)
        rospy.loginfo("Tracking started!")
        while not rospy.is_shutdown():
	    trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'head_1', rospy.Duration())
	    head = np.array([trans[0], trans[1], trans[2]])

	    trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'neck_1', rospy.Duration())
            neck = np.array([trans[0], trans[1], trans[2]])

    	    trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'right_shoulder_1', rospy.Duration())
            
            right_shoulder = np.array([trans[0], trans[1], trans[2]])

            trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'right_elbow_1', rospy.Duration())
            right_elbow = np.array([trans[0], trans[1], trans[2]])

            trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'right_hand_1', rospy.Duration())
            right_hand = np.array([trans[0], trans[1], trans[2]])

            trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'left_shoulder_1', rospy.Duration())
            left_shoulder = np.array([trans[0], trans[1], trans[2]])

            trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'left_elbow_1', rospy.Duration())
            left_elbow = np.array([trans[0], trans[1], trans[2]])

            trans, rot = self.tf.lookupTransform('/openni_depth_frame', 'left_hand_1', rospy.Duration())
            left_hand = np.array([trans[0], trans[1], trans[2]])
		
	    # left shoulder roll/pitch
            left_shoulder_elbow = left_elbow - left_shoulder
            left_shoulder_neck = right_shoulder - left_shoulder

            left_shoulder_elbow = left_shoulder_elbow / np.linalg.norm(left_shoulder_elbow)
            left_shoulder_neck = left_shoulder_neck / np.linalg.norm(left_shoulder_neck)

            left_shoulder_angle_roll = math.acos(left_shoulder_elbow.dot(left_shoulder_neck)) - math.pi/2

            left_shoulder_angle_pitch = -1.0 * math.asin(left_shoulder_elbow[2])

	    # right shoulder roll/pitch
	    right_shoulder_elbow = right_elbow - right_shoulder
            right_shoulder_neck = left_shoulder - right_shoulder

            right_shoulder_elbow = right_shoulder_elbow / np.linalg.norm(right_shoulder_elbow)
            right_shoulder_neck = right_shoulder_neck / np.linalg.norm(right_shoulder_neck)

            right_shoulder_angle_roll = -1.0 * math.acos(right_shoulder_elbow.dot(right_shoulder_neck)) + math.pi/2

            right_shoulder_angle_pitch = -1.0 * math.asin(right_shoulder_elbow[2])

	    # left elbow roll
 	    left_elbow_hand = left_hand - left_elbow
            left_elbow_shoulder = left_shoulder - left_elbow
            left_elbow_hand = left_elbow_hand / np.linalg.norm(left_elbow_hand)
            left_elbow_shoulder = left_elbow_shoulder / np.linalg.norm(left_elbow_shoulder)
            left_elbow_angle_roll =  math.acos(left_elbow_shoulder.dot(left_elbow_hand)) - math.pi
	    
	    # right elbow roll
            right_elbow_hand = right_hand - right_elbow
            right_elbow_shoulder = right_shoulder - right_elbow
            right_elbow_hand = right_elbow_hand / np.linalg.norm(right_elbow_hand)
            right_elbow_shoulder = right_elbow_shoulder / np.linalg.norm(right_elbow_shoulder)
            right_elbow_angle_roll = -1.0 * math.acos(right_elbow_hand.dot(right_elbow_shoulder)) + math.pi

	    # right elbow yaw
	    right_elbow_angle_yaw = right_elbow_hand[2] / math.sin(right_elbow_angle_roll) * math.pi/2
            if right_elbow_angle_yaw > 1.5:
                right_elbow_angle_yaw = 1.5
            elif right_elbow_angle_yaw < -1.5:
                right_elbow_angle_yaw = -1.5

	    # left elbow roll
            left_elbow_angle_yaw = left_elbow_hand[2] / math.sin(left_elbow_angle_roll) * math.pi/2
            if left_elbow_angle_yaw > 1.5:
                left_elbow_angle_yaw = 1.5
            elif left_elbow_angle_yaw < -1.5:
                left_elbow_angle_yaw = -1.5
	    
    	    # head yaw
	    head_neck = head - neck
	    head_neck = head_neck / np.linalg.norm(head_neck)
	    head_yaw = math.pi/2 - math.asin(head_neck[2])

	    jdl = jointdatalist()
            jd_pitch = jointdata(Part=String("LShoulderPitch"), Angles=Float32(left_shoulder_angle_pitch))
	    jd_roll = jointdata(Part=String("LShoulderRoll"), Angles=Float32(left_shoulder_angle_roll))
            jdl.joints.append(jd_pitch)
	    jdl.joints.append(jd_roll)

	    jd_pitch = jointdata(Part=String("RShoulderPitch"), Angles=Float32(right_shoulder_angle_pitch))
	    jd_roll = jointdata(Part=String("RShoulderRoll"), Angles=Float32(right_shoulder_angle_roll))
            jdl.joints.append(jd_pitch)
	    jdl.joints.append(jd_roll)

	    jd_roll = jointdata(Part=String("LElbowRoll"), Angles=Float32(left_elbow_angle_roll))
	    jd_yaw = jointdata(Part=String("LElbowYaw"), Angles=Float32(left_elbow_angle_yaw))
	    jdl.joints.append(jd_roll)
	    jdl.joints.append(jd_yaw)

	    jd_roll = jointdata(Part=String("RElbowRoll"), Angles=Float32(right_elbow_angle_roll))
	    jd_yaw = jointdata(Part=String("RElbowYaw"), Angles=Float32(right_elbow_angle_yaw))
	    jdl.joints.append(jd_roll)
	    jdl.joints.append(jd_yaw)

	    jd_yaw = jointdata(Part=String("HeadPitch"), Angles=Float32(head_yaw))
	    # jd_roll = jointdata(Part=String("HeadPitch"), Angles=Float32(head_pitch))

            jdl.joints.append(jd_yaw)
	    # jdl.joints.append(jd_roll)

            self.pub_trans.publish(jdl)
            rospy.loginfo('pitch:{0}'.format(head_yaw/math.pi*180))
            r.sleep()

if __name__ == '__main__':
    try:
        Tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass