"""Utility functions for finding and loading Unitree models."""

from pathlib import Path
import mujoco


def find_unitree_root() -> Path:
    """
    Dynamically find the Unitree installation directory.
    
    Checks multiple possible locations and returns the first one that exists.
    
    Returns:
        Path to Unitree root directory
        
    Raises:
        FileNotFoundError: If Unitree directory cannot be found
    """
    possible_roots = [
        Path(r"D:\Unitree"),
        Path(r"E:\Unitree"),
        Path(r"F:\Unitree"),
        Path(r"U:\Unitree"),
    ]
    
    for root in possible_roots:
        xml = root / "unitree_mujoco" / "unitree_robots" / "g1" / "scene_23dof.xml"
        if xml.exists():
            print(f"Found Unitree directory: {root}")
            return root
    
    raise FileNotFoundError("Unitree directory not found. Checked: " + ", ".join(str(r) for r in possible_roots))


def load_g1_model() -> mujoco.MjModel:
    """
    Load the G1 robot model from scene_23dof.xml.
    
    Returns:
        Loaded MuJoCo model
    """
    unitree_root = find_unitree_root()
    model_path = unitree_root / "unitree_mujoco" / "unitree_robots" / "g1" / "scene_23dof.xml"
    
    print(f"Loading model: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    
    return model
