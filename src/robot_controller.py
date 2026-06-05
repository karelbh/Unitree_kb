"""Base robot controller class for MuJoCo simulations."""

import time
import math
import mujoco
import mujoco.viewer
import numpy as np
from typing import Optional


class RobotController:
    """Base class for controlling robots in MuJoCo simulations."""

    def __init__(self, model_path: str):
        """
        Initialize the robot controller.

        Args:
            model_path: Path to the MuJoCo XML model file
        """
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        
    def print_model_info(self):
        """Print information about the loaded model."""
        print(f"Model loaded: {self.model}")
        print(f"Joints: {self.model.njnt}")
        print(f"Actuators: {self.model.nu}")
        
        print("\n=== JOINTS ===")
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"{i:2d}  {name}")

        print("\n=== ACTUATORS ===")
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f"{i:2d}  {name}")

    def find_actuator_by_name(self, actuator_name: str) -> Optional[int]:
        """
        Find actuator ID by name.

        Args:
            actuator_name: Name of the actuator

        Returns:
            Actuator ID if found, None otherwise
        """
        actuator_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            actuator_name
        )
        return actuator_id if actuator_id >= 0 else None

    def set_actuator_target(self, actuator_id: int, target: float):
        """
        Set target position for an actuator.

        Args:
            actuator_id: ID of the actuator
            target: Target position value
        """
        if 0 <= actuator_id < self.model.nu:
            self.data.ctrl[actuator_id] = target

    def set_actuator_sine_motion(
        self,
        actuator_id: int,
        amplitude: float = 0.25,
        frequency: float = 0.2,
        offset: float = 0.0
    ):
        """
        Apply sinusoidal motion pattern to an actuator.

        Args:
            actuator_id: ID of the actuator
            amplitude: Motion amplitude in radians
            frequency: Motion frequency in Hz
            offset: DC offset
        """
        self._sine_params = {
            "actuator_id": actuator_id,
            "amplitude": amplitude,
            "frequency": frequency,
            "offset": offset,
            "start_time": time.time()
        }

    def _update_sine_motion(self):
        """Update sinusoidal motion (internal use)."""
        if not hasattr(self, "_sine_params"):
            return

        params = self._sine_params
        elapsed = time.time() - params["start_time"]
        target = params["offset"] + params["amplitude"] * math.sin(
            2.0 * math.pi * params["frequency"] * elapsed
        )
        self.set_actuator_target(params["actuator_id"], target)

    def step(self, dt: Optional[float] = None):
        """
        Execute one simulation step.

        Args:
            dt: Optional time step (uses model default if None)
        """
        self._update_sine_motion()
        mujoco.mj_step(self.model, self.data)

    def launch_viewer(self):
        """Launch MuJoCo viewer."""
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        return self.viewer

    def close_viewer(self):
        """Close MuJoCo viewer."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None

    def run_simulation(
        self,
        duration: float = 10.0,
        use_viewer: bool = True,
        timestep: float = 0.002
    ):
        """
        Run the simulation.

        Args:
            duration: Simulation duration in seconds
            use_viewer: Whether to display the viewer
            timestep: Simulation timestep
        """
        if use_viewer:
            self.launch_viewer()

        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                self.step()
                
                if use_viewer and self.viewer:
                    self.viewer.sync()
                
                time.sleep(timestep)
        finally:
            if use_viewer:
                self.close_viewer()
