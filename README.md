# LINEMOD + Keypoint Pose Pipeline

This project provides a template generation pipeline and a ROS 2 detector node that combines:
- OpenCV LINEMOD (color templates) for detection
- ORB keypoints for parallel detection and pose disambiguation via PnP
- OpenGL rendering for template generation (color + depth)

## Overview

The flow is split into two stages:

1) **Template generation**
   - Render a CAD model from multiple viewpoints with OpenGL.
   - Save LINEMOD templates and keypoint templates.

2) **Runtime detection**
   - Run LINEMOD on live images (ROS 2 node).
   - Run keypoint matching in parallel and estimate pose with PnP.
   - Use the keypoint pose to resolve in-plane ambiguity.

Key output files created by template generation:
- `linemod_templates.yml.gz` (LINEMOD templates)
- `linemod_tempPosFile.bin` (template poses)
- `keypoint_templates.yml.gz` (keypoint templates + 3D points)

## Template Generation

The template generator renders both color and depth images and extracts templates.

Command:
```bash
cmake -H. -B build
cmake --build build --config Release --target Template_Generator
./build/Template_Generator
```

Template generation is controlled by `linemod_settings.yml`:
- `video width`, `video height`: render size
- `camera fx/fy/cx/cy`: camera intrinsics
- `in plane rotation angle step`: in-plane rotation sampling
- `distance start/stop/step`: viewpoint radius sampling
- `icosahedron subdivisions`: number of viewpoints per radius

## LINEMOD Detection

LINEMOD templates are matched against the color image. The detection result is refined with depth
(if enabled in `linemod_settings.yml`).

Important settings:
- `only use color modality`: set to `0` to enable depth modality during template creation
- `detector threshold`: LINEMOD match threshold
- `use depth improvement`: enable depth consistency check
- `depth offset`: depth correction (mm)

## Keypoint Detection and Pose

The keypoint detector uses ORB features:
- Keypoints are extracted from rendered color templates.
- The corresponding depth image is used to back-project keypoints into 3D (object coordinates).
- At runtime, ORB matches are used to estimate pose via `solvePnPRansac`.

Outputs:
- 2D detection boxes (from keypoint inliers)
- 6D pose (rotation + translation) from RGB keypoints

This helps disambiguate 180-degree in-plane flips that often occur with LINEMOD.

## ROS 2 Node (Live or Bag)

Build:
```bash
cd ros2
colcon build --symlink-install --packages-select linemod_detector
```

Run:
```bash
source /opt/ros/jazzy/setup.bash
source /home/posedetection/Documents/Dianis_CPS/LINEMOD/ros2/install/setup.bash
cd /home/posedetection/Documents/Dianis_CPS/LINEMOD
ros2 run linemod_detector linemod_detector_node --ros-args \
  -p class_name:=milk_carton.ply \
  -p color_topic:=/camera/color/image_raw/compressed \
  -p depth_topic:=/camera/depth/image_raw/compressed \
  -p display:=true
```

Replay a bag:
```bash
ros2 bag play /home/posedetection/Documents/Dianis_CPS/LINEMOD/bags/milk_carton_test2
```

## OpenGL Settings Used

These are hard-coded in the renderer:
- **GL context:** OpenGL 3.3 Core profile
- **Depth buffer:** GL_DEPTH_COMPONENT32
- **Color buffer format:** GL_R16 for depth render target
- **Depth range:**
  - Near = 100.0
  - Far = 10000.0
- **Projection:**
  - `glm::perspective(fov, width/height, 100.0f, 10000.0f)`
- **Depth shader:** linearizes `gl_FragCoord.z` to millimeters

The depth shader uses:
```
LinearizeDepth(depth) = (2 * near * far) / (far + near - z * (far - near))
```

The rendered depth is stored as 16-bit unsigned and used for template creation.

## File Outputs

Required at runtime:
- `linemod_settings.yml`
- `linemod_templates.yml.gz`
- `linemod_tempPosFile.bin`
- `keypoint_templates.yml.gz`
- `models/*.ply`
- `shader/*`

## Notes

- For faster template generation, increase `in plane rotation angle step` and lower
  `icosahedron subdivisions`.
- If the OpenCV display window does not appear, ensure you are running in a GUI session
  (not headless SSH).
