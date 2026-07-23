# MediaPipe Hand Landmarker model

`hand_tracker_node` requires the following model file:

```text
models/hand_landmarker.task
```

Download the official MediaPipe model:

```bash
cd ~/cobot_ws/src/hand_teleop
mkdir -p models
wget -O models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

The node can also use an external model path:

```bash
ros2 run hand_teleop hand_tracker_node --ros-args \
  -p model_path:=/absolute/path/to/hand_landmarker.task
```
