#!/usr/bin/env python3

import rospy
import rospkg

rospack = rospkg.RosPack()
template = rospack.get_path("gazebo_radiation_plugins")+"/custom_models/SI/template/radiation_source_template.sdf"
sources_folder = rospack.get_path("gazebo_radiation_plugins")+"/custom_models/SI/models/source/"

sources = rospy.get_param("sources")

outputfile = "source_"

#print sources

for i in range(0,len(sources)):
    pose_set = False
    with open(template) as f:
        with open(sources_folder+"/"+outputfile+str(i)+".sdf", "w") as f1:
            for line in f:
                if "topic" in line:
                    f1.write("<topic>"+outputfile+str(i)+"/"+ sources[outputfile+str(i)]['type'] +"</topic> \n")
                elif ("pose" in line) & (pose_set == False):
                    x = sources[outputfile+str(i)]['x']
                    y = sources[outputfile+str(i)]['y']
                    z = sources[outputfile+str(i)]['z']
                    #print x,y,z
                    f1.write("<pose>{} {} {} 0 0 0</pose> \n".format(str(x),str(y),str(z+0.1)))
                    pose_set = True
                else:
                    f1.write(line)