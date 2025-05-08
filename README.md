# Grasp_Project_IFPT

To Test First Part:

1) docker run -it --privileged     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --     
   device=/dev/bus/usb:/dev/bus/usb     --device=/dev/video0:/dev/video0     -v /dev/shm:/dev/shm     realsense-ros2

2)  ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    enable_pointcloud:=true \
    enable_depth:=true \
    enable_color:=true \
    filters:=pointcloud \
    pointcloud.enable:=true \
    pointcloud.texture_stream:=RS2_STREAM_COLOR \
    align_depth.enable:=true


3) open two other containers

   -docker exec -it CONTAINER_NUMBER bash

4) Open RVIZ2 in one and test in the other.

5) To test
   
ros2 topic list

/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/color/points
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_color
/clicked_point
/goal_pose
/initialpose
/parameter_events
/rosout
/tf
/tf_static

ros2 topic info /camera/camera/depth/color/points

Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
Subscription count: 1



6) in RVIZ2

/*************************************************************************************************/
Set the Fixed Frame: In the Rviz2 window, set the fixed frame to camera_link or camera_depth_frame, depending on your setup.

(Add PointCloud2 Display:)

In the Rviz2 left sidebar, click on the "Add" button.

Choose PointCloud2 from the list of displays.

In the Topic field of the new display, select /camera/camera/depth/color/points.

This should allow you to visualize the point cloud in Rviz2.

/************************************************************************************************/
