# 🤖🍓 Stretch MuJoCo Manipulation

**Vernacular voice-controlled robotic strawberry harvesting using Hello Robot Stretch 2 in MuJoCo simulation**

[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-orange.svg)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Hackathon](https://img.shields.io/badge/AI%20for%20Bharat-Hackathon-red.svg)](https://ai4bharat.org)

> **Kisan-Drishti (Farmer's Vision)**: Empowering farmers with voice-controlled robotic precision agriculture in their native languages.

## 🎯 Overview

**Kisan-Drishti** is a vernacular-first robotic farming system that enables farmers to control agricultural robots using voice commands in their native languages (Hindi, Kannada, Tamil, etc.). This repository contains the MuJoCo physics simulation for precision strawberry harvesting using the Hello Robot Stretch 2.

### Why This Matters

- **🌾 Accessibility**: Farmers can control robots in their own language without technical training
- **🎯 Precision**: Physics-based simulation ensures accurate manipulation before real-world deployment
- **💰 Cost-Effective**: Reduces labor costs and increases yield through automated harvesting
- **🌍 Scalability**: Simulation-tested algorithms can deploy to real Stretch robots nationwide

### Built For

**AI for Bharat Hackathon 2026** - Track 6: AI for Communities, Access & Public Impact

---

## ✨ Features

### 🗣️ **Vernacular Voice Control**
- Multi-language support: Hindi (हिंदी), Kannada (ಕನ್ನಡ), Tamil (தமிழ்)
- Natural language commands: "उत्तर की पंक्ति से स्ट्रॉबेरी तोड़ो" → Robot executes

### 🤖 **Advanced Robotics**
- Full Stretch 2 robot simulation with accurate kinematics
- 8 DOF control: mobile base, lift, arm extension, wrist, gripper, head
- Physics-based magnet gripper for gentle berry picking

### 🔬 **Realistic Physics**
- MuJoCo-powered dynamics with contact simulation
- Gravity, friction, and collision detection
- Berries with realistic mass (15g) and material properties

### 📸 **Visual Feedback**
- Real-time camera feeds from robot's head
- Mission completion screenshots
- Multiple viewpoints: main, top-down, side

### 🎮 **Interactive Simulation**
- Live MuJoCo viewer for debugging
- Adjustable parameters (speed, thresholds, trajectories)
- Step-by-step execution monitoring

---

## 🎬 Demo

### Video Demonstration

[Upload your screencast here]

### Example Workflow

```
Farmer (WhatsApp Voice): "उत्तर पंक्ति से दूसरी स्ट्रॉबेरी तोड़ो"
                          (Pick second strawberry from north row)
           ↓
      [n8n Workflow]
           ↓
   [Google Antigravity Agent]
           ↓
    [MuJoCo Simulation]
           ↓
      Robot executes:
      ✓ Extends arm to berry
      ✓ Activates magnet gripper
      ✓ Retracts with berry
      ✓ Captures success photo
           ↓
  [Photo sent back via WhatsApp]
``

## 🏗️ Architecture

```

---

## 🚀 Installation

### Prerequisites

- **Python 3.8+**
- **MuJoCo 3.0+**
- **Ubuntu 24.04** (or compatible Linux)

### Step 1: Clone Repository

```bash
git clone https://github.com/BhushanGatty/stretch-mujoco-manipulation.git
cd stretch-mujoco-manipulation
```

### Step 2: Install MuJoCo

```bash
# Download MuJoCo
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.0/mujoco-3.1.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.1.0-linux-x86_64.tar.gz
sudo mv mujoco-3.1.0 /usr/local/mujoco

# Set environment variables (add to ~/.bashrc)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/mujoco/lib
```

### Step 3: Install Python Dependencies

```bash
pip install --break-system-packages mujoco numpy imageio pillow
```

### Step 4: Download Stretch Robot Model

```bash
# Clone MuJoCo Menagerie for Stretch 2 model
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

### Step 5: Verify Installation

```bash
python -c "import mujoco; print(f'MuJoCo {mujoco.__version__} installed!')"
```

---

## ⚡ Quick Start

### Run Basic Harvest Simulation

```bash
python harvest_simple.py
```

**Expected Output:**
```
======================================================================
🍓 HARVESTING: berry2
======================================================================

📍 Berry positions after settling:
  berry2: [ 0.00,  0.30,  0.34]

🦾 Reaching for berry at [0.00, 0.30, 0.34]
  Required reach: 0.40m
  ✓ Reached berry! Distance: 0.089m

🧲 Activating magnet...
  ✅ MAGNET ON!

⬆️  Retracting...
📸 Taking photo...

======================================================================
✅ SUCCESS
Photo: berry2_harvest.png
======================================================================
```

### Test Physics Settling

```bash
python test_berry_physics_v2.py
```

### Try Different Berries

Edit `harvest_simple.py`:
```python
TARGET = "berry4"  # Try berry1, berry2, berry3, berry4, or berry5
```

---

## 📖 Usage

### Basic Harvesting

```python
from harvest_simple import SimpleStrawberryPicker

# Initialize picker
picker = SimpleStrawberryPicker("scene_very_close.xml")

# Harvest a berry
success, photo = picker.harvest("berry2")

if success:
    print(f"✅ Harvested! Photo: {photo}")
```

### Custom Berry Positions

Edit `scene_very_close.xml`:
```xml
<body name="my_berry" pos="0.0 0.25 0.8">
    <freejoint/>
    <inertial pos="0 0 0" mass="0.020" diaginertia="0.00002 0.00002 0.00002"/>
    <geom name="my_berry_geom" type="sphere" size="0.04" material="berry_mat" 
          friction="2.0 0.1 0.05" condim="3" contype="2" conaffinity="15"/>
</body>
```

### Adjust Control Parameters

```python
# Modify reach controller gains
self.data.ctrl[ACT["lift"]] = np.clip(height_error * 5.0, -0.1, 0.1)
self.data.ctrl[ACT["arm_extend"]] = np.clip(reach_error * 6.0, -1.0, 1.0)

# Change magnet pick threshold
success = self.magnet_pick(berry_name, viewer, threshold=0.20)  # Default: 0.18
```
---

## 🔧 Technical Details

### Robot Specifications

**Hello Robot Stretch 2**
- **DOF**: 8 controllable actuators
- **Max Reach**: 0.52m (arm extension)
- **Lift Range**: 0.0 - 1.1m
- **Base**: Differential drive with turn and forward control
- **Gripper**: Magnetic attachment (weld constraint simulation)
- **Cameras**: RGB + Depth in head assembly

### Actuator Mapping

| Index | Name | Function |
|-------|------|----------|
| 0 | `forward` | Linear base movement |
| 1 | `turn` | Rotational base movement |
| 2 | `lift` | Vertical arm translation (Z-axis) |
| 3 | `arm_extend` | Telescoping arm extension |
| 4 | `wrist_yaw` | Gripper rotation |
| 5 | `grip` | Finger open/close |
| 6 | `head_pan` | Horizontal camera rotation |
| 7 | `head_tilt` | Vertical camera rotation |

### Physics Parameters

```python
# Simulation
timestep = 0.005s
integrator = "implicitfast"
gravity = [0, 0, -9.8]

# Berry Properties
mass = 0.020 kg (20g)
radius = 0.04m (4cm)
friction = [2.0, 0.1, 0.05]
contact_stiffness = solimp="0.95 0.99 0.001"

# Magnet Constraint
type = "weld"
active = False (until proximity trigger)
threshold = 0.18m
```

### Control Loop

```python
# Proportional controller for arm extension
while distance_to_target > threshold:
    error = target_pos - current_pos
    
    # Height control
    lift_ctrl = clip(height_error * 5.0, -0.1, 0.1)
    
    # Reach control
    extend_ctrl = clip(reach_error * 6.0, -1.0, 1.0)
    
    # Apply and step
    data.ctrl[ACT["lift"]] = lift_ctrl
    data.ctrl[ACT["arm_extend"]] = extend_ctrl
    mujoco.mj_step(model, data)
```

---

## 🗺️ Roadmap

### ✅ Completed (v1.0)
- [x] MuJoCo environment setup
- [x] Stretch 2 robot integration
- [x] Physics-based berry simulation
- [x] Magnet gripper implementation
- [x] Reach-and-pick controller
- [x] Camera capture system

### 🚧 In Progress (v1.1)
- [ ] Mobile base navigation (debugging turn actuator)
- [ ] Multi-berry sequential harvesting
- [ ] Collision avoidance
- [ ] Improved gripper contact detection

### 🔮 Future (v2.0)
- [ ] Voice command integration (n8n + Antigravity)
- [ ] Real-time WhatsApp feedback
- [ ] Multi-language NLP (Hindi/Kannada/Tamil)
- [ ] ROS2 bridge for real Stretch deployment
- [ ] Reinforcement learning for optimal trajectories
- [ ] Vision-based berry detection (YOLOv8)
- [ ] Multi-robot coordination

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### Issues We Need Help With

- 🐛 **Bug**: Mobile base turn actuator not responding
- 🎯 **Enhancement**: Implement visual berry detection
- 📚 **Documentation**: Add Hindi/Kannada code comments
- 🧪 **Testing**: Unit tests for controller functions

### How to Contribute

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Development Setup

```bash
# Install development dependencies
pip install pytest black flake8

# Run tests
pytest tests/

# Format code
black controllers/ tests/

# Lint
flake8 controllers/ tests/
```
