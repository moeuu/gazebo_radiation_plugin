#!/usr/bin/env python3

import rospy
import rospkg
import os

rospack = rospkg.RosPack()

sources_folder = rospack.get_path("gazebo_radiation_plugins")+"/custom_models/SI/models/source/"

sources = rospy.get_param("sources")

for i in sources: 
    s = sources_folder+i
    os.system("rosrun gazebo_ros spawn_model -file {}.sdf -sdf -model {}".format(s,i))