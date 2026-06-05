#!/usr/bin/env python3
"""
Basic motion control example.

This script demonstrates how to use the RobotController to apply
sinusoidal motion to a robot actuator.
"""

import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.robot_controller import RobotController
from src.utils import find_xml_models, list_available_models


def main():
    # Find available models
    models = find_xml_models()
    
    if not models:
        print("No models found. Please place model XML files in the models/ directory")
        list_available_models()
        return

    # Use first available model
    model_path = list(models.values())[0]
    print(f"Using model: {model_path}")

    # Create controller
    controller = RobotController(model_path)
    
    # Print model information
    controller.print_model_info()

    # Apply sinusoidal motion to first actuator (if available)
    if controller.model.nu > 0:
        print("\nApplying sinusoidal motion to actuator 0...")
        controller.set_actuator_sine_motion(
            actuator_id=0,
            amplitude=0.5,
            frequency=0.5
        )
        
        # Run simulation with viewer
        print("Running simulation for 10 seconds...")
        controller.run_simulation(duration=10.0, use_viewer=True)
    else:
        print("No actuators found in model")


if __name__ == "__main__":
    main()
