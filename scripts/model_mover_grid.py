#!/usr/bin/env python3

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
import time
import numpy as np
import tf

class Mover(object):

    def __init__(self):

        self.i = 0
        self.filename = "/home/morita/Radiation_distribution_machine_learning/data/result1.csv"
        f = open(self.filename, "w")
        f.close()
        rospy.init_node('set_pose')
        rospy.Subscriber("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, self.callback)
        
        for i in range(6):
            self.topic_name = "/radiation_source_plugin/source_" + str(i) + "/gamma"
            self.sub = rospy.Subscriber(self.topic_name, Simulated_Radiation_Msg)
            f = open(self.filename, "a")
            f.write("{},{},{},{}\n".format(self.sub.pose.position.x,self.sub.pose.position.y,self.sub.pose.position.z,self.sub.value))
            f.close()


        state_msg = ModelState()
        state_msg.model_name = 'sensor_0'
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1.0
        self.x = 0
        self.y = 0



        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.i = 0      
            self.x = (self.i%14)*1 + 0.5
            self.y = (np.floor(self.i/14))*1 + 0.5
            while self.i < 10*14+1:
                
                quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
                state_msg.pose.orientation.x = quaternion[0]
                state_msg.pose.orientation.y = quaternion[1]
                state_msg.pose.orientation.z = quaternion[2]
                state_msg.pose.orientation.w = quaternion[3]
                state_msg.pose.position.x = self.x
                state_msg.pose.position.y = self.y
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )

                rospy.wait_for_message("/radiation_sensor_plugin/sensor_0", Simulated_Radiation_Msg, timeout=None)
                    
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def callback(self,data):
        print(data.value)
        if (data.pose.position.x == self.x)&(self.y == data.pose.position.y):
            self.i+=1
            self.x = (self.i%14)*1 + 0.5
            self.y = (np.floor(self.i/14))*1 + 0.5
            if self.x == 0.0:
                self.x = 0.0001
            if self.y == 0.0:
                self.y = 0.0001
            f = open(self.filename, "a")
            f.write("{},{},{},{}\n".format(data.pose.position.x,data.pose.position.y,data.pose.position.z,data.value))
            f.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        m = Mover()
        m.run()
    except rospy.ROSInterruptException:
        pass