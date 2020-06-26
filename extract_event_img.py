#!/usr/bin/env python
# See LICENSE for licensing information.
#
# Copyright (c) 2020-2020
# Written by daehyun.kim@gatech.edu
# All rights reserved
#

import os
import sys
from os import listdir
import pathlib

# Read the user defined configurations
def read_configs():
    configs = {}

    with open("configs.txt", 'r') as infile:
        for line in infile:
            line = line.split('\n')[0].split(':')
            var_name = line[0].strip()
            data = line[1].strip()
            configs[var_name] = data

    # Read the list of the videos
    videos_list = [f for f in listdir(configs["videos_path"])]

    return configs, videos_list


# Generate the frame pictures based on the video file.
def gen_frames(configs, video_name):

    # Set the video file path
    file_path = configs["videos_path"] + video_name

    # Generate the directory for the frames
    frame_path = configs["output_path"] + "frames/" + video_name               
    os.system("mkdir -p " + frame_path)

    # Generate the frames from the video
    current_path = str(pathlib.Path().absolute())
    os.system("ffmpeg -i " + file_path + " " + frame_path + "/frames_%010d.png")
    os.chdir("/home/xshe6/sim_ws/src/rpg_esim/event_camera_simulator/esim_ros/")
    os.system("python scripts/generate_stamps_file.py -i " + frame_path + \
        " -r 1200.0")
    os.chdir(current_path)


# Generate the bag file from the frame pictures
def gen_bagfile(configs, video_name):
    
    # Set the frames' path
    frame_path = configs["output_path"] + "frames/" + video_name
    
    # Generate the option list
    options = "--data_source=" + configs["data_source"]
    options = options + " --path_to_output_bag=./tmp/sample.bag"
    options = options + " --path_to_data-folder=" + frame_path
    options = options + " --ros_publisher_frame_rate=" + \
        configs["ros_publisher_frame_rate"]
    options = options + " --exposure_time_ms=" + configs["exposure_time_ms"]
    options = options + " --use_log_image=" + configs["use_log_image"]
    options = options + " --log_eps=" + configs["log_eps"]
    options = options + " --contrast_threshold_pos=" + \
        configs["contrast_threshold_pos"]
    options = options + " --contrast_threshold_neg=" + \
        configs["contrast_threshold_neg"]

    # Simulate events with ESIM
    os.system("rosrun esim_ros esim_node " + options)


# Generate the event camera pictures from the bag file
def gen_event_frames(configs, video_name):
    
    # Generate the export.launch file
    bag_path = str(pathlib.Path().absolute()) + "/tmp/sample.bag"
    sec_per_frame = round(1/float(configs["ros_publisher_frame_rate"]), 4)

    with open("export.launch", 'w') as outfile:
        outfile.write("<launch>\n")
        outfile.write("  <node pkg=\"rosbag\" type=\"play\" name=\"rosbag\" required=\"true\" ")
        outfile.write("args=\"" + bag_path + "\"/>\n")
        outfile.write("  <node name=\"extract\" pkg=\"image_view\" type=\"extract_images\" ")
        outfile.write("respawn=\"false\" required=\"true\" output=\"screen\" cwd=\"ROS_HOME\">\n")
        outfile.write("    <remap from=\"image\" to=\"/dvs_rendering\"/>\n")
        outfile.write("    <param name=\"sec_per_frame\" value=\"" + str(sec_per_frame) + "\"/>\n")
        outfile.write("  </node>\n")
        outfile.write("</launch>\n")

    # Run the event camera frame extract script
    os.system("roslaunch export.launch")

    # Move the frame pictures to output directory
    output_path = configs["output_path"] + "event_frames/" + video_name
    os.system("mkdir -p " + output_path)
    os.system("mv ~/.ros/frame*.jpg " + output_path + '/')


############################################################
############################################################
############################################################
############################################################


if __name__ == "__main__":
    configs, videos_list = read_configs()

    for video_name in videos_list:
        gen_frames(configs, video_name)
        gen_bagfile(configs, video_name)
        gen_event_frames(configs, video_name)
