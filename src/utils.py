"""Utility functions for robot control."""

import os
from pathlib import Path
from typing import List, Optional
import mujoco


def find_xml_models(search_dirs: Optional[List[str]] = None) -> dict:
    """
    Search for available XML model files.

    Args:
        search_dirs: Directories to search (uses default locations if None)

    Returns:
        Dictionary mapping model names to file paths
    """
    if search_dirs is None:
        search_dirs = [
            "./models",
            "../models",
            "models",
            "./",
        ]

    models = {}
    
    for search_dir in search_dirs:
        if not os.path.exists(search_dir):
            continue
            
        for root, dirs, files in os.walk(search_dir):
            for file in files:
                if file.endswith(".xml"):
                    full_path = os.path.join(root, file)
                    model_name = file[:-4]  # Remove .xml extension
                    models[model_name] = full_path

    return models


def load_model_info(model_path: str) -> dict:
    """
    Load and extract information about a MuJoCo model.

    Args:
        model_path: Path to the XML model file

    Returns:
        Dictionary with model information
    """
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        joints = []
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joints.append(name)

        actuators = []
        for i in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            actuators.append(name)

        return {
            "path": model_path,
            "joints": joints,
            "actuators": actuators,
            "num_joints": model.njnt,
            "num_actuators": model.nu,
        }
    except Exception as e:
        return {"error": str(e), "path": model_path}


def list_available_models(search_dirs: Optional[List[str]] = None):
    """
    List all available models with their information.

    Args:
        search_dirs: Directories to search
    """
    models = find_xml_models(search_dirs)
    
    print(f"\nFound {len(models)} models:")
    print("-" * 60)
    
    for model_name, model_path in sorted(models.items()):
        info = load_model_info(model_path)
        
        if "error" in info:
            print(f"\n{model_name}:")
            print(f"  Path: {model_path}")
            print(f"  Error: {info['error']}")
        else:
            print(f"\n{model_name}:")
            print(f"  Path: {model_path}")
            print(f"  Joints: {info['num_joints']}")
            print(f"  Actuators: {info['num_actuators']}")
