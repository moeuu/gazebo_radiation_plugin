#!/usr/bin/env python3

import os
import rospy 
import rospkg 
import random
import subprocess

class auto_mover(object):
    def __init__(self) -> None:
        self.source_folder_path = '../custom_models/SI/models/source'
        self.source_template_path = "../custom_models/SI/template/radiation_source_template.sdf"
        self.random_iter = random.randint(1, 10)
        for i in range(10000):
            self.random_iter = random.randint(1, 10)
            for j in range(self.random_iter):
                self.create_source(j)
            if(i != 9999):
                self.delete_source()

    def delete_source(self):
        files = os.listdir(self.source_folder_path)
        for file in files:
            file_path = os.path.join(self.source_folder_path, file)
            if os.path.isfile(file_path):
                os.remove(file_path)

    def create_source(self, i):
        for i in range(self.random_iter):
            self.rad_sdf = "../custom_models/SI/models/source/source_" + str(i) + ".sdf"
            with open(self.source_template_path, 'r') as template_file:
                radiation_sdf = template_file.read()
            #名前の置き換え
            radiation_sdf_name = radiation_sdf.replace('{name}', str(i))
            #ランダムな座標生成
            x = random.randint(0,13) + 0.5
            y = random.randint(0, 9) + 0.5
            radation_sdf_x = radiation_sdf_name.replace('{x}', str(x))
            radiation_sdf_xy = radation_sdf_x.replace('{y}', str(y))

            with open(self.rad_sdf, "w") as rad_sdf:
                rad_sdf.write(radiation_sdf_xy)


if __name__ == "__main__":
    try:
        m = auto_mover()
    except rospy.ROSInterruptException:
        pass