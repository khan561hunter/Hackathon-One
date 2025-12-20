---
id: capstone-vla-manipulation
title: Voice-Controlled Object Manipulation (Capstone)
sidebar_label: VLA Manipulation
sidebar_position: 7
---

# یوزر اسٹوری 4: Voice-Controlled Object Manipulation (Capstone Project)

## پروجیکٹ کا مقصد

ایک مکمل system بنائیں جہاں humanoid robot voice commands کی بنیاد پر objects manipulate کرتا ہے، تمام پچھلے User Stories کے concepts کو integrate کرتے ہوئے۔

## سیکھنے کے مقاصد

اس capstone project کے اختتام تک، آپ:
- OpenAI Whisper استعمال کرتے ہوئے voice-to-text implement کریں گے
- Language models کے ساتھ cognitive planning کریں گے
- Robotic manipulation اور grasping implement کریں گے
- تمام components کو مکمل system میں integrate کریں گے
- End-to-end testing اور debugging کریں گے

## System Architecture

```
┌─────────────┐     Audio      ┌──────────────┐    Text     ┌─────────────┐
│ Microphone  │ ─────────────> │   Whisper    │ ──────────> │ Language    │
│             │                │ Voice-to-Text│             │ Model (LLM) │
└─────────────┘                └──────────────┘             └─────────────┘
                                                                    │
                                                                    │ Plan
                                                                    ↓
┌─────────────┐                ┌──────────────┐             ┌─────────────┐
│   Camera    │ ─────────────> │   Object     │ ──────────> │   Grasp     │
│  (Vision)   │   RGB-D        │  Detection   │  Poses      │  Planning   │
└─────────────┘                └──────────────┘             └─────────────┘
                                                                    │
                                                                    │ Trajectory
                                                                    ↓
                                                            ┌─────────────┐
                                                            │    Robot    │
                                                            │  Controller │
                                                            └─────────────┘
```

## پیشگی ضروریات

- تمام پچھلے User Stories (1-3) مکمل
- Python 3.8+
- ROS 2 Humble
- NVIDIA GPU (inference کے لیے)
- OpenAI API key (اختیاری، GPT کے لیے)

## Part 1: Voice-to-Text (Whisper)

### Whisper Model Setup

```bash
# Whisper install کریں
pip install openai-whisper

# Model download کریں (پہلے run پر automatic)
# Models: tiny, base, small, medium, large
```

### Voice Command Processing

```python
import whisper
import pyaudio
import numpy as np

class VoiceCommandProcessor:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.audio = pyaudio.PyAudio()

    def listen_and_transcribe(self):
        # Record audio
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        print("Listening...")
        frames = []
        for i in range(0, int(16000 / 1024 * 3)):  # 3 seconds
            data = stream.read(1024)
            frames.append(data)

        # Transcribe
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0

        result = self.model.transcribe(audio_data)
        return result["text"]
```

## Part 2: Language Understanding

### Command Parsing

```python
class CommandParser:
    def parse_command(self, text):
        text_lower = text.lower()

        # Identify action
        if "pick up" in text_lower or "grab" in text_lower:
            action = "grasp"
        elif "place" in text_lower or "put" in text_lower:
            action = "place"
        elif "move" in text_lower:
            action = "move"
        else:
            return None

        # Extract object
        objects = ["cup", "bottle", "box", "ball"]
        target_object = None
        for obj in objects:
            if obj in text_lower:
                target_object = obj
                break

        # Extract location (if any)
        locations = ["table", "shelf", "ground"]
        target_location = None
        for loc in locations:
            if loc in text_lower:
                target_location = loc
                break

        return {
            "action": action,
            "object": target_object,
            "location": target_location
        }
```

### LLM Integration (اختیاری)

```python
from openai import OpenAI

class LLMPlanner:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

    def plan_manipulation(self, command):
        prompt = f"""
        You are a robot task planner. Given the command: "{command}"

        Generate a step-by-step plan:
        1. Identify the target object
        2. Plan approach trajectory
        3. Execute grasp
        4. Move to destination
        5. Release object

        Output as JSON with steps.
        """

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )

        return response.choices[0].message.content
```

## Part 3: Object Detection

### YOLO for Object Detection

```python
from ultralytics import YOLO
import cv2

class ObjectDetector:
    def __init__(self):
        self.model = YOLO('yolov8n.pt')

    def detect_objects(self, image):
        results = self.model(image)

        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[class_id]

                detections.append({
                    "class": class_name,
                    "confidence": confidence,
                    "bbox": [x1, y1, x2, y2]
                })

        return detections
```

## Part 4: Grasp Planning

### Grasp Pose Estimation

```python
class GraspPlanner:
    def plan_grasp(self, object_pose, object_type):
        # Top-down grasp for most objects
        grasp_pose = object_pose.copy()
        grasp_pose[2] += 0.15  # Approach from above

        # Adjust gripper orientation
        if object_type == "cup":
            # Side grasp
            grasp_approach = "side"
        elif object_type == "box":
            # Top grasp
            grasp_approach = "top"
        else:
            # Default top grasp
            grasp_approach = "top"

        return grasp_pose, grasp_approach
```

### Inverse Kinematics

```python
import modern_robotics as mr

def calculate_ik(target_pose, current_joint_angles):
    # Robot's kinematic chain
    Slist = get_screw_axes()  # Robot-specific
    M = get_home_transformation()  # End-effector home pose

    # Compute IK
    thetalist, success = mr.IKinSpace(
        Slist, M, target_pose,
        current_joint_angles,
        eomg=0.001, ev=0.001
    )

    return thetalist if success else None
```

## Part 5: Motion Execution

### Trajectory Planning

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryExecutor:
    def execute_trajectory(self, waypoints, duration=5.0):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        num_points = len(waypoints)
        time_step = duration / num_points

        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start.sec = int(i * time_step)
            point.time_from_start.nanosec = int((i * time_step % 1) * 1e9)
            traj.points.append(point)

        self.trajectory_pub.publish(traj)
```

## Part 6: مکمل Integration

### Main Control Loop

```python
class VLAManipulationSystem:
    def __init__(self):
        self.voice_processor = VoiceCommandProcessor()
        self.command_parser = CommandParser()
        self.object_detector = ObjectDetector()
        self.grasp_planner = GraspPlanner()
        self.trajectory_executor = TrajectoryExecutor()

    def run(self):
        while True:
            # 1. Listen for command
            text = self.voice_processor.listen_and_transcribe()
            print(f"Heard: {text}")

            # 2. Parse command
            parsed = self.command_parser.parse_command(text)
            if not parsed:
                print("Command not understood")
                continue

            # 3. Detect object
            image = self.get_camera_image()
            detections = self.object_detector.detect_objects(image)

            # 4. Find target object
            target = self.find_object(detections, parsed["object"])
            if not target:
                print(f"Object {parsed['object']} not found")
                continue

            # 5. Plan grasp
            grasp_pose, approach = self.grasp_planner.plan_grasp(
                target["pose"], target["class"])

            # 6. Execute
            waypoints = self.plan_motion(grasp_pose)
            self.trajectory_executor.execute_trajectory(waypoints)

            print("Task completed!")
```

## Demo Scenarios

### Scenario 1: "Pick up the red cup"

```
Voice → "Pick up the red cup"
  ↓
Whisper → "pick up the red cup"
  ↓
Parser → {action: "grasp", object: "cup", color: "red"}
  ↓
Vision → Detect red cup at position (0.3, 0.2, 0.8)
  ↓
Grasp Plan → Side grasp approach
  ↓
IK → Joint angles calculated
  ↓
Execute → Robot picks up cup
```

### Scenario 2: "Place the box on the table"

```
Voice → "Place the box on the table"
  ↓
Parser → {action: "place", object: "box", location: "table"}
  ↓
Vision → Detect box (already grasped) + table surface
  ↓
Place Plan → Target pose on table
  ↓
Execute → Robot places box
```

## Evaluation Criteria

1. **Voice Recognition Accuracy**: > 90%
2. **Object Detection Accuracy**: > 85%
3. **Grasp Success Rate**: > 80%
4. **End-to-End Task Completion**: > 75%
5. **Average Execution Time**: < 30 seconds

## کلیدی سبق

1. **Multimodal Integration**: Voice، vision، اور manipulation کو combine کرنا
2. **Pipeline Design**: Modular components آسانی سے debug ہوتے ہیں
3. **Error Handling**: ہر stage میں robust fallbacks
4. **Real-Time Performance**: Latency minimize کرنا user experience کے لیے critical
5. **System Integration**: تمام پچھلے User Stories یہاں converge ہوتی ہیں

## مبارکباد!

آپ نے Physical AI & Humanoid Robotics کا مکمل journey مکمل کر لیا ہے:
- ✅ ROS 2 Fundamentals
- ✅ Joint Control
- ✅ Navigation
- ✅ AI Training
- ✅ Voice-Controlled Manipulation

## اگلے قدم

- اپنے خود کے custom applications بنائیں
- Open-source robotics community میں contribute کریں
- Cutting-edge research papers explore کریں
- اپنا physical robot بنائیں!

## حوالہ جات

- Radford, A., et al. (2022). Robust Speech Recognition via Large-Scale Weak Supervision. *arXiv:2212.04356*.
- Dosovitskiy, A., et al. (2020). An Image is Worth 16x16 Words. *ICLR 2021*.
- Brohan, A., et al. (2023). RT-2: Vision-Language-Action Models. *arXiv:2307.15818*.
