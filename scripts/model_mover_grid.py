#!/usr/bin/env python3

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
import time
import numpy as np
import tf
import sys

class Mover(object):

    def __init__(self):

        self.i = 0
        if len(sys.argv) > 1:
            self.file_num = sys.argv[1]
            print("file_number:", self.file_num)
        else:
            print("File number isn't provided.")
        self.filename = "/home/morita/src/Radiation_distribution_machine_learning/data/rad_cnt/result" + str(self.file_num) + ".csv"
        f = open(self.filename, "w")
        f.close()
        rospy.init_node('set_pose')

        self.topics = rospy.get_published_topics()
        self.topic_names = [topic[0] for topic in self.topics]
        self.j = 0
        while "/radiation_source_plugin/source_" + str(self.j) + "/gamma" in self.topic_names:
            self.topic_name = "/radiation_source_plugin/source_" + str(self.j) + "/gamma"
            self.sub = rospy.wait_for_message(self.topic_name, Simulated_Radiation_Msg, timeout=None)
            f = open(self.filename, "a")
            f.write("{},{},{},{},{}\n".format(self.sub.pose.position.x,self.sub.pose.position.y,self.sub.pose.position.z,self.sub.value,"source_data"))
            f.close()
            print("{},{},{},{},{}".format(self.sub.pose.position.x,self.sub.pose.position.y,self.sub.pose.position.z,self.sub.value,self.topic_name))
            self.j += 1


        state_msg = ModelState()
        state_msg.model_name = 'sensor_0'
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1.0
        self.x = 0
        self.y = 0
        self.rad_pos = False

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.i = 0      
            self.x = (self.i%140)*0.1 + 0.05
            self.y = (np.floor(self.i/140))*0.1 + 0.05
            while self.i < 100*140:
                
                quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
                state_msg.pose.orientation.x = quaternion[0]
                state_msg.pose.orientation.y = quaternion[1]
                state_msg.pose.orientation.z = quaternion[2]
                state_msg.pose.orientation.w = quaternion[3]
                state_msg.pose.position.x = self.x
                state_msg.pose.position.y = self.y
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )

                self.i += 1
                self.x = (self.i%140)*0.1 + 0.05
                self.y = (np.floor(self.i/140))*0.1 + 0.05
                if self.x == 0.0:
                    self.x = 0.0001
                if self.y == 0.0:
                    self.y = 0.0001

                data = rospy.wait_for_message("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, timeout=None)
                print(data.value)
                f = open(self.filename, "a")
                f.write("{},{},{},{}\n".format(data.pose.position.x,data.pose.position.y,data.pose.position.z,data.value))
                f.close()
                    
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        m = Mover()
    except rospy.ROSInterruptException:
        pass