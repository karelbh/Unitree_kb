# Setup Guide

## Prerequisites

- Python 3.8 or higher
- pip (Python package manager)
- Virtual environment support (venv)

## Installation Steps

### 1. Clone Repository

```bash
git clone https://github.com/your-username/Unitree_kb.git
cd Unitree_kb
```

### 2. Create Virtual Environment

**Windows:**
```bash
python -m venv venv
venv\Scripts\activate
```

**macOS/Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Download Robot Models

You need to obtain the Unitree robot models:

1. Visit [Unitree Robotics GitHub](https://github.com/unitreerobotics)
2. Clone or download the `unitree_mujoco` repository
3. Extract the robot models to the `models/` directory

Example structure:
```
models/
├── g1/
│   ├── scene_23dof.xml
│   └── other files...
├── h1/
│   ├── scene.xml
│   └── other files...
└── ...
```

### 5. Verify Installation

**List available models:**
```bash
python examples/find_models.py
```

**Run basic example:**
```bash
python examples/basic_motion.py
```

## Troubleshooting

### MuJoCo Rendering Issues

If you encounter rendering issues on Windows:

1. Ensure graphics drivers are up to date
2. Update MuJoCo: `pip install --upgrade mujoco`

### Model Not Found

If scripts cannot find models:

1. Verify `models/` directory exists in project root
2. Place `.xml` files in the `models/` directory
3. Run `python examples/find_models.py` to verify

### Import Errors

If you get import errors when running examples:

1. Ensure virtual environment is activated
2. Verify all dependencies installed: `pip install -r requirements.txt`
3. Check that you're running from the project root directory

## Advanced Setup

### Using Existing Virtual Environment

If you have the Unitree venv already set up, you can activate it directly:

```bash
# Windows Unitree venv example
C:\UnitreeVenvs\unitree-DESKTOP-IFF8GVP\Scripts\activate
```

### Development Installation

For development with editable installs:

```bash
pip install -e .[dev]
```

## Next Steps

- Check out [examples/](../examples/) for usage examples
- Read [API.md](API.md) for detailed API documentation
- Run `python examples/basic_motion.py` to test your setup
