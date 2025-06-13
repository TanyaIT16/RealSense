# RealSense Image Capture

This repository includes a simple ROS 2 example for saving images from an Intel® RealSense™ D435 camera.

## Requirements
- ROS 2 Humble
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- OpenCV

## Building

After cloning this repository into a ROS 2 workspace, build the package from the
workspace root and source the environment:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage
1. Launch the RealSense camera node (as you already do):
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```
2. In another terminal, run the image saver using `ros2 run`:
   ```bash
   ros2 run realsense_image_saver capture_images
   ```
   By default the node subscribes to `/color/image_raw` and saves images under the `images/` directory. You can change the topic or output directory using ROS parameters:
   ```bash
   ros2 run realsense_image_saver capture_images --ros-args \
       -p image_topic:=/my/image/topic -p output_dir:=/path/to/save
   ```

To capture images less frequently, set `save_interval_sec` to the desired number
of seconds. For example, to save one image every 30 seconds:

```bash
ros2 run realsense_image_saver capture_images --ros-args \
    -p save_interval_sec:=30
```

`ros2 run` requires that the package has been built and that the environment
is sourced, as shown in the *Building* section above. If you see `No executable
found`, rebuild the package and source `install/setup.bash` again. You can
verify the installation with:

```bash
ros2 pkg executables realsense_image_saver
```

The captured images will be stored sequentially as `frame_XXXXXX.png`.
