#!/bin/bash

source ~/.bashrc
find_pkg=$(rospack find colored_pointcloud)
find_ws=$find_pkg/../..
bag_name=5
gnome-terminal --title="tp_repub" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud repub_3dRebuild_tp.launch bag_name:=$bag_name"
gnome-terminal --title="colored_pc" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud colored_pointcloud_node.launch color_distance:=0.5 camera_tp:=/usb_cam/image_$bag_name lidar_tp:=/livox_$bag_name/lidar param_file_name:=calib_result.yaml"

