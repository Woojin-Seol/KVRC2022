#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class caster():
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        self.model_name = rospy.get_param("/robot_name", '/')
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.base_cb)
        self.marker_pub = rospy.Publisher('/drone_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(2)

        self.br = tf.TransformBroadcaster()
        self.marker = Marker()
        self.marker.mesh_resource = "package://khnp_competition/resources/drone.dae"
        self.marker.type = 10
        self.marker.mesh_use_embedded_materials = True
        self.marker.header.frame_id = "map"
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1

    def base_cb(self, msg):
        self.timestamp = rospy.Time.now()
        for i in range(len(msg.name)):
            if msg.name[i] == self.model_name:
                current_pose = msg.pose[i]
                self.br.sendTransform((current_pose.position.x, current_pose.position.y, current_pose.position.z),\
(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w),\
self.timestamp,"base_link","map")
                self.marker.header.stamp = rospy.Time.now()
                self.marker.pose = current_pose
                self.marker_pub.publish(self.marker)
        self.br.sendTransform((0.05, 0.0, 0.0), (0.5,-0.5,0.5,-0.5), self.timestamp, "d435i/depth_camera_link","base_link")
        return

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
