# LINEMOD + Keypoint Pose Pipeline

This repository implements a practical 6D pose-estimation pipeline for known objects using:
- **OpenCV LINEMOD** templates for fast detection
- **ORB keypoints + PnP** for pose disambiguation and refinement
- **OpenGL rendering** for offline template generation from CAD models (`.ply`)
- An optional **ROS 2 node** for live runtime detection

---

## 1) Problem solved and how

### The problem
Classic template-based detection such as LINEMOD can:
- Detect objects quickly,
- But struggle with **pose ambiguity** (especially in-plane rotations and symmetric views), and
- Often needs good template coverage to work reliably.

### The approach in this repo
This project addresses those issues by splitting the system into two stages:

1) **Offline template generation**
   - Render a CAD model from many viewpoints using OpenGL.
   - Generate:
     - LINEMOD templates,
     - Keypoint templates with associated 3D points.

2) **Online runtime detection**
   - Run LINEMOD on incoming RGB (and optionally depth).
   - In parallel, run ORB keypoint matching.
   - Use `solvePnPRansac` on keypoint matches to resolve ambiguous poses.
   - Optionally refine with ICP using depth.

In short: **LINEMOD proposes detections; keypoints + PnP stabilize the pose.**

---

## 2) Repository structure (what each part does)

### Top-level build and entrypoint
- `CMakeLists.txt`  
  Builds the core C++ code and the offline template generator executable (`Template_Generator`).
- `templateGeneration.cpp`  
  Minimal entrypoint that runs the offline template generation pipeline.
- `linemod_settings.yml`  
  Central configuration: camera intrinsics, template sampling distances/angles, detector thresholds, and model location/extension.

### Core pipeline modules
- `include/` + `src/` contain the main logic:
  - `TemplateGenerator` orchestrates offline template generation.
  - `PoseDetection` orchestrates runtime detection and pose estimation.
  - `HighLevelLineMOD` wraps LINEMOD template creation and detection.
  - `KeypointDetector` builds keypoint templates and runs ORB matching + PnP pose estimation.
  - `OpenGLRender` renders color/depth views from models to generate templates and support refinement.
  - `HighLevelLinemodIcp` provides optional ICP-based depth refinement and best-pose selection.

### Models, shaders, and artifacts
- `models/`  
  Input CAD meshes (`.ply`) and per-model metadata (`.yml`).
- `shader/`  
  GLSL shaders used by the renderer for color and depth rendering.
- Generated artifacts (created by offline template generation):
  - `linemod_templates.yml.gz`
  - `linemod_tempPosFile.bin`
  - `keypoint_templates.yml.gz`

### ROS 2 runtime node
- `ros2/src/linemod_detector/` contains a ROS 2 package that:
  - Subscribes to compressed RGB and depth images,
  - Decodes/resizes them to match the configured camera parameters,
  - Calls the core `PoseDetection` pipeline.

---

## 3) How the offline template generation works (simple diagram)

```text
[linemod_settings.yml]
          │
          ▼
 [TemplateGenerator]
          │
          ▼
     [models/*.ply]
          │
          ▼
 [OpenGL rendering]
 (many views: color+depth)
          │
          ▼
   [mask from depth]
          │
     ┌────┴─────┐
     ▼          ▼
[LINEMOD]   [ORB keypoints]
templates     + 3D points
     │          │
     └────┬─────┘
          ▼
 [template files written]
 linemod_templates.yml.gz
 linemod_tempPosFile.bin
 keypoint_templates.yml.gz
```

This flow corresponds directly to the main loop in `TemplateGenerator::run()`.

---

## 4) How runtime detection works (simple diagram)

```text
[RGB + Depth input]
        │
        ▼
[ROS2 linemod_detector_node]
(decode + resize)
        │
        ▼
   [PoseDetection]
        │
   ┌────┼──────────────────────────┐
   ▼    ▼                          ▼
[LINEMOD match]         [Keypoint ORB + PnP]
   │                                │
   └──────────────┬─────────────────┘
                  ▼
        [Optional ICP refinement]
                  ▼
            [Final 6D pose]
```

Key behavior: if keypoint PnP succeeds, it can override the LINEMOD pose to resolve ambiguity.

---

## 5) Installation and build instructions

### 5.1 Core C++ build (offline template generator)

Clone the repository and enter it:

```bash
git clone <REPO_URL>
cd linemod_keypoint_tracking
```

From the repository root:

```bash
cmake -S . -B build
cmake --build build --config Release --target Template_Generator
```

Run the offline generator:

```bash
./build/Template_Generator
```

The executable entrypoint is `templateGeneration.cpp`.

### 5.2 ROS 2 node build (online detection)

From the `ros2/` directory (inside the cloned repo):

```bash
cd ros2
colcon build --symlink-install --packages-select linemod_detector
```

This builds `linemod_detector_node`.

---

## 6) How to use the software

### Step A — Configure camera and sampling
Edit `linemod_settings.yml`:
- Camera intrinsics: `camera fx/fy/cx/cy`
- Rendering size: `video width/height`
- Sampling: `distance start/stop/step`, `in plane rotation angle step`, `icosahedron subdivisions`
- Model path and extension: `model folder`, `model file ending`

### Step B — Provide models
Place your `.ply` model(s) in `models/`. They are discovered automatically based on:
- `model folder: models/`
- `model file ending: ".ply"`

Per-model metadata (color range + symmetry) can be provided via `models/<name>.yml`.

### Step C — Generate templates offline
Run:

```bash
./build/Template_Generator
```

This produces the template files used at runtime.

### Step D — Run runtime detection (ROS 2)
Example:

```bash
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash

ros2 run linemod_detector linemod_detector_node --ros-args \
  -p class_name:=milk_carton.ply \
  -p color_topic:=/camera/color/image_raw/compressed \
  -p depth_topic:=/camera/depth/image_raw/compressed \
  -p display:=true
```

The node decodes compressed images and forwards them into `PoseDetection::detect(...)`.

---

## 7) Example results / what to expect

When everything is configured correctly, you can expect:

1) **Templates are created offline**
   - LINEMOD templates and keypoint templates are saved to disk.

2) **Runtime detections produce poses**
   - The pipeline finds candidate poses via LINEMOD,
   - Optionally refines with ICP,
   - And can override with a more stable keypoint PnP pose if available.

3) **A visual overlay appears (if enabled)**
   - The coordinate axes are drawn onto the color image and displayed in a window.

---

## 8) Clean-code checklist for submission

To keep the repository presentation-ready:

- Avoid committing temporary outputs such as build folders:
  - `build/`
  - `ros2/build/`, `ros2/install/`, `ros2/log/`
- Keep only required runtime artifacts and inputs:
  - `linemod_settings.yml`
  - generated templates (`*.yml.gz`, `*.bin`)
  - models and shaders

---

## 9) Key files to read first

If you have limited time, start here:
- Offline pipeline: `src/TemplateGenerator.cpp`
- Runtime pipeline: `src/PoseDetection.cpp`
- ROS 2 entrypoint: `ros2/src/linemod_detector/src/linemod_detector_node.cpp`
