from pathlib import Path
import mujoco


def find_project_root() -> Path:
    """
    Najde kořen aktuálního GitHub projektu Unitree_kb.
    Hlavní pracovní adresář:
    D:\\work\\tata\\projekty\\Unitree_kb
    """

    project_root = Path(__file__).resolve().parent

    model_path = (
        project_root
        / "unitree_mujoco"
        / "unitree_robots"
        / "g1"
        / "scene_23dof.xml"
    )

    if model_path.exists():
        return project_root

    possible_roots = [
        Path(r"D:\work\tata\projekty\Unitree_kb"),
        Path(r"D:\Unitree"),
        Path(r"E:\Unitree"),
        Path(r"F:\Unitree"),
        Path(r"U:\Unitree"),
    ]

    for root in possible_roots:
        candidate = (
            root
            / "unitree_mujoco"
            / "unitree_robots"
            / "g1"
            / "scene_23dof.xml"
        )
        if candidate.exists():
            return root

    checked = [model_path]
    for root in possible_roots:
        checked.append(
            root
            / "unitree_mujoco"
            / "unitree_robots"
            / "g1"
            / "scene_23dof.xml"
        )

    raise FileNotFoundError(
        "Unitree model not found. Checked:\n"
        + "\n".join(str(p) for p in checked)
    )


def get_g1_model_path() -> Path:
    root = find_project_root()
    return (
        root
        / "unitree_mujoco"
        / "unitree_robots"
        / "g1"
        / "scene_23dof.xml"
    )


def load_g1_model():
    model_path = get_g1_model_path()
    print("Loading model:", model_path)
    return mujoco.MjModel.from_xml_path(str(model_path))