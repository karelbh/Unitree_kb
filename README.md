# Unitree_kb

Knowledge base and control library for Unitree humanoid robots (G1, H1, A1, B1).

MuJoCo physics simulation with robot control examples and utilities.

## Overview

This project provides:
- **Robot Control Scripts**: Controllers for Unitree robots G1, H1, and other models
- **MuJoCo Simulations**: Physics-based simulations of robot dynamics
- **Documentation**: Setup guides, tutorials, and API documentation
- **Examples**: Working examples for common tasks (motion control, inverse kinematics, etc.)

## Supported Robots

- **G1 EDU**: 23 DOF humanoid robot
- **H1**: Humanoid robot
- **A1**: Quadruped robot
- **B1**: Quadruped robot

## Requirements

- Python 3.8+
- MuJoCo 3.0+
- NumPy
- SciPy (optional, for advanced control)

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/your-username/Unitree_kb.git
cd Unitree_kb
```

### 2. Create virtual environment
```bash
python -m venv venv
venv\Scripts\activate  # Windows
source venv/bin/activate  # macOS/Linux
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Place robot models
Download Unitree robot models and place them in `models/` directory:
```
models/
├── g1/
│   └── scene_23dof.xml
├── h1/
│   └── scene.xml
└── ...
```

## Quick Start

### Run G1 Control Example
```bash
python src/g1_control.py
```

### List Available Models
```bash
python src/find_models.py
```

## Project Structure

```
Unitree_kb/
├── src/                      # Main source code
│   ├── __init__.py
│   ├── robot_controller.py   # Base controller class
│   ├── g1_control.py         # G1-specific control
│   └── utils.py              # Utility functions
├── models/                   # Robot XML/URDF models
├── examples/                 # Example scripts
│   ├── basic_motion.py
│   └── trajectory_control.py
├── docs/                     # Documentation
│   ├── SETUP.md
│   └── API.md
├── tests/                    # Unit tests
├── requirements.txt          # Python dependencies
├── setup.py                  # Package setup
├── LICENSE                   # MIT License
├── .gitignore               # Git ignore rules
└── README.md                # This file
```

## Usage Examples

### Basic Motion Control
```python
from src.robot_controller import RobotController

# Initialize controller
controller = RobotController(model_path="models/g1/scene_23dof.xml")

# Apply sinusoidal motion to shoulder
controller.set_actuator_sine_motion(
    actuator_id=0,
    amplitude=0.5,
    frequency=1.0
)

# Run simulation
controller.run()
```

### Custom Control Loop
```python
import mujoco

model = mujoco.MjModel.from_xml_path("models/g1/scene_23dof.xml")
data = mujoco.MjData(model)

for step in range(1000):
    data.ctrl[0] = 0.5  # Set actuator 0 target
    mujoco.mj_step(model, data)
```

## Documentation

- [Setup Guide](docs/SETUP.md)
- [API Reference](docs/API.md)
- [Contributing](CONTRIBUTING.md)

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md)

## License

MIT License - see [LICENSE](LICENSE)

## References

- [Unitree Robotics](https://www.unitreerobotics.com)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [G1 EDU Documentation](https://www.unitreerobotics.com/products)

## Author

Created for educational and research purposes.

## Support

For issues, questions, or suggestions, please open an issue on GitHub.
