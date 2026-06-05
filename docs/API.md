# API Reference

## RobotController

Main class for controlling robots in MuJoCo simulations.

### Constructor

```python
RobotController(model_path: str)
```

- `model_path`: Path to MuJoCo XML model file

### Methods

#### `print_model_info()`

Print information about the loaded model including joints and actuators.

```python
controller.print_model_info()
```

#### `find_actuator_by_name()`

Find actuator ID by name.

```python
actuator_id = controller.find_actuator_by_name("shoulder_pitch")
```

**Returns:** `int` or `None`

#### `set_actuator_target()`

Set target position for an actuator.

```python
controller.set_actuator_target(actuator_id=0, target=0.5)
```

- `actuator_id`: ID of the actuator
- `target`: Target position value

#### `set_actuator_sine_motion()`

Apply sinusoidal motion to an actuator.

```python
controller.set_actuator_sine_motion(
    actuator_id=0,
    amplitude=0.5,
    frequency=1.0,
    offset=0.0
)
```

**Parameters:**
- `actuator_id`: ID of the actuator
- `amplitude`: Motion amplitude in radians (default: 0.25)
- `frequency`: Motion frequency in Hz (default: 0.2)
- `offset`: DC offset (default: 0.0)

#### `step()`

Execute one simulation step.

```python
controller.step()
```

#### `launch_viewer()`

Launch MuJoCo viewer for visualization.

```python
viewer = controller.launch_viewer()
```

**Returns:** Viewer object

#### `close_viewer()`

Close the MuJoCo viewer.

```python
controller.close_viewer()
```

#### `run_simulation()`

Run the simulation for a specified duration.

```python
controller.run_simulation(
    duration=10.0,
    use_viewer=True,
    timestep=0.002
)
```

**Parameters:**
- `duration`: Simulation duration in seconds (default: 10.0)
- `use_viewer`: Display viewer window (default: True)
- `timestep`: Simulation step size (default: 0.002)

## Utility Functions

### `find_xml_models()`

Search for available XML model files.

```python
from src.utils import find_xml_models

models = find_xml_models(search_dirs=["models", "./"])
```

**Returns:** Dictionary mapping model names to file paths

### `load_model_info()`

Load and extract information about a MuJoCo model.

```python
from src.utils import load_model_info

info = load_model_info("models/g1/scene_23dof.xml")
print(f"Joints: {info['num_joints']}")
print(f"Actuators: {info['num_actuators']}")
```

**Returns:** Dictionary with model information

### `list_available_models()`

List all available models with their information.

```python
from src.utils import list_available_models

list_available_models()
```

## Examples

### Basic Motion Control

```python
from src.robot_controller import RobotController

# Initialize
controller = RobotController("models/g1/scene_23dof.xml")

# Apply motion
controller.set_actuator_sine_motion(
    actuator_id=0,
    amplitude=0.5,
    frequency=1.0
)

# Run simulation
controller.run_simulation(duration=10.0)
```

### Custom Control Loop

```python
from src.robot_controller import RobotController
import mujoco

controller = RobotController("models/g1/scene_23dof.xml")
controller.launch_viewer()

try:
    for i in range(5000):
        # Custom control logic
        for j in range(controller.model.nu):
            controller.data.ctrl[j] = 0.0
        
        controller.step()
        controller.viewer.sync()
finally:
    controller.close_viewer()
```

## Error Handling

```python
try:
    controller = RobotController(model_path)
except FileNotFoundError:
    print(f"Model not found: {model_path}")
except Exception as e:
    print(f"Error loading model: {e}")
```
