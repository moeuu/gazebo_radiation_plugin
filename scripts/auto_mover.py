#!/usr/bin/env python3

import os
import rospy 
import rospkg 
import random
import subprocess
import time
import numpy as np

class auto_mover(object):
    def __init__(self) -> None:
        self.source_folder_path = '../custom_models/SI/models/source'
        self.source_template_path = "../custom_models/SI/template/radiation_source_template.sdf"
        for i in range(100):
            roscore_process = subprocess.Popen(["roscore"])
            #gazebo起動
            gazebo_process = subprocess.Popen(["rosrun", "gazebo_radiation_plugins", "gazebo", "--verbose"])
            time.sleep(5)
            #random_rad個の点線源生成
            self.random_rad = random.randint(1, 10)
            self.create_source_yaml()
            self.create_source()
            os.system("rosparam load ../custom_models/SI/configs/radiation.yaml")
            os.system("rosparam load ../custom_models/SI/configs/sensors.yaml")
            time.sleep(2)
            #線源読み込み
            os.system("rosrun gazebo_radiation_plugins load_radiation_sources.py")
            #センサー読み込み
            time.sleep(2)
            os.system("rosrun gazebo_ros spawn_model -file ../custom_models/SI/models/mysensor.sdf -sdf -model sensor_0")
            #i番目のresult作成
            time.sleep(2)
            self.mover = "rosrun gazebo_radiation_plugins model_mover_grid.py " + str(i+22)
            os.system(self.mover)
            if(i != 3):
                self.delete_source()
            roscore_process.terminate()
            time.sleep(3)
            gazebo_process.terminate()
            os.system("pkill gzserver")
            os.system("killall gzclient")
            time.sleep(5)

    def delete_source(self):
        files = os.listdir(self.source_folder_path)
        for file in files:
            file_path = os.path.join(self.source_folder_path, file)
            if os.path.isfile(file_path):
                os.remove(file_path)

    def create_source(self):
        #random_rad個の点線源生成
        for i in range(self.random_rad):
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
    
    def create_source_yaml(self):
        with open("../custom_models/SI/template/radiation_template.yaml", "r") as rad_yaml_temp:
            rad_yaml_i = rad_yaml_temp.read()
        self.rad_yaml = "../custom_models/SI/configs/radiation.yaml"
        for i in range(self.random_rad):
            random_value = np.random.uniform(500, 10000)
            rad_yaml_i = rad_yaml_i + "\n  source_{}:\n    noise: 0.0\n    type: gamma\n    units: Sv/h\n    value: {}".format(i, random_value)
            with open(self.rad_yaml, "w") as rad_yaml:
                rad_yaml.write(rad_yaml_i)


if __name__ == "__main__":
    try:
        m = auto_mover()
    except rospy.ROSInterruptException:
        pass