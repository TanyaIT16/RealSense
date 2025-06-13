# RealSense Image Capture

This repository includes a simple ROS 2 example for saving images from an Intel® RealSense™ D435 camera.

## Requirements
- ROS 2 Humble
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- OpenCV

## Building


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
   By default the node subscribes to `/camera/color/image_raw` and saves images
   under the `images/` directory. Check the available image topics with
   `ros2 topic list` and override `image_topic` if needed. Common topics from
   the RealSense driver include `/camera/color/image_raw`,
   `/camera/depth/image_rect_raw`, or namespaced versions like
   `/camera/camera/color/image_raw`. You can also change
   the output directory using ROS parameters:
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


The captured images will be stored sequentially as `frame_XXXXXX.png`.
