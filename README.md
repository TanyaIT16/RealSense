# RealSense Image Capture

This repository includes a simple ROS 2 example for saving images from an Intel® RealSense™ D435 camera.

## Requirements
- ROS 2 Humble
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- OpenCV

## Usage
1. Launch the RealSense camera node (as you already do):
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```
2. In another terminal, run the image saver script:
   ```bash
   python3 realsense_image_saver/realsense_image_saver/capture_images.py
   ```
   By default the node subscribes to `/color/image_raw` and saves images under the `images/` directory. You can change the topic or output directory using ROS parameters:
   ```bash
   ros2 run realsense_image_saver capture_images --ros-args \
       -p image_topic:=/my/image/topic -p output_dir:=/path/to/save
   ```

The captured images will be stored sequentially as `frame_XXXXXX.png`.
