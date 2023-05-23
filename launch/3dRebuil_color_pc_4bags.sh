#!/bin/bash

source ~/.bashrc
find_pkg=$(rospack find colored_pointcloud)
find_ws=$find_pkg/../..

gnome-terminal --title="tp_repub" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud repub_3dRebuild_tp_4bag.launch"
sleep 1s
gnome-terminal --title="colored_pc" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud colored_pointcloud_node.launch color_distance:=0.5 camera_tp:=/usb_cam/image_1 lidar_tp:=/livox_1/lidar param_file_name:=calib_result.yaml ns_:=scene1"
sleep 1s
gnome-terminal --title="colored_pc" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud colored_pointcloud_node.launch color_distance:=0.5 camera_tp:=/usb_cam/image_2 lidar_tp:=/livox_2/lidar param_file_name:=calib_result.yaml ns_:=scene2"
sleep 1s
gnome-terminal --title="colored_pc" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud colored_pointcloud_node.launch color_distance:=0.5 camera_tp:=/usb_cam/image_3 lidar_tp:=/livox_3/lidar param_file_name:=calib_result.yaml ns_:=scene3"
sleep 1s
gnome-terminal --title="colored_pc" -- bash -c "source $find_ws/devel/setup.bash;roslaunch colored_pointcloud colored_pointcloud_node.launch color_distance:=0.5 camera_tp:=/usb_cam/image_5 lidar_tp:=/livox_5/lidar param_file_name:=calib_result.yaml ns_:=scene4"