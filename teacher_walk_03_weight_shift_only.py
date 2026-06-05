import time
import math
from pathlib import Path

import mujoco
import mujoco.viewer


def find_unitree_root():
    possible_roots = [
        Path(r"F:\Unitree"),
        Path(r"U:\Unitree"),
        Path(r"D:\Unitree"),
        Path(r"E:\Unitree"),
    ]

    for root in possible_roots:
        xml = root / "unitree_mujoco" / "unitree_robots" / "g1" / "scene_23dof.xml"
        if xml.exists():
            print(f"Používám složku Unitree: {root}")
            return root

    raise FileNotFoundError("Nenalezena složka Unitree.")


UNITREE_ROOT = find_unitree_root()
XML_PATH = UNITREE_ROOT / "unitree_mujoco" / "unitree_robots" / "g1" / "scene_23dof.xml"

BASE_Z = 0.78


def set_joint(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name
    )

    if joint_id < 0:
        print(f"Kloub nenalezen: {joint_name}")
        return False

    qpos_addr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_addr] = value
    return True


def prepare_standing_pose(model, data):
    """
    Kinematický základní postoj.
    Nohy zůstávají v klidu, robot neřeší fyzikální rovnováhu.
    """

    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    if model.nq >= 7:
        # poloha těla
        data.qpos[0] = 0.0
        data.qpos[1] = 0.0
        data.qpos[2] = BASE_Z

        # orientace těla quaternion
        data.qpos[3] = 1.0
        data.qpos[4] = 0.0
        data.qpos[5] = 0.0
        data.qpos[6] = 0.0

    # Nohy – pevný mírně pokrčený postoj
    set_joint(model, data, "left_hip_pitch_joint", -0.12)
    set_joint(model, data, "left_hip_roll_joint", 0.03)
    set_joint(model, data, "left_hip_yaw_joint", 0.0)
    set_joint(model, data, "left_knee_joint", 0.28)
    set_joint(model, data, "left_ankle_pitch_joint", -0.14)
    set_joint(model, data, "left_ankle_roll_joint", -0.03)

    set_joint(model, data, "right_hip_pitch_joint", -0.12)
    set_joint(model, data, "right_hip_roll_joint", -0.03)
    set_joint(model, data, "right_hip_yaw_joint", 0.0)
    set_joint(model, data, "right_knee_joint", 0.28)
    set_joint(model, data, "right_ankle_pitch_joint", -0.14)
    set_joint(model, data, "right_ankle_roll_joint", 0.03)

    # Trup – základ
    set_joint(model, data, "waist_yaw_joint", 0.0)
    set_joint(model, data, "waist_roll_joint", 0.0)
    set_joint(model, data, "waist_pitch_joint", 0.0)

    # Ruce v klidové poloze
    set_joint(model, data, "left_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "left_shoulder_roll_joint", 0.15)
    set_joint(model, data, "left_elbow_joint", 0.30)

    set_joint(model, data, "right_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "right_shoulder_roll_joint", -0.15)
    set_joint(model, data, "right_elbow_joint", 0.30)

def apply_weight_shift_only(model, data, t):
    """
    Vizuální ukázka přenášení horní části těla.
    
    Důležité:
    - neposouváme floating base robota,
    - nehýbeme kyčlemi,
    - nehýbeme koleny,
    - nehýbeme kotníky,
    - chodidla mají zůstat na místě,
    - pohybuje se pouze trup/pas a ruce jako protiváha.

    Toto ještě není plná inverzní kinematika přenášení váhy.
    Je to bezpečný mezikrok.
    """

    frequency = 0.25
    phase = 2.0 * math.pi * frequency * t
    s = math.sin(phase)

    # Pouze náklon horní části těla vlevo/vpravo.
    waist_roll = 0.12 * s

    # Malá rotace trupu pro lepší vizuální efekt.
    waist_yaw = 0.03 * s

    # Ruce jako protiváha.
    left_shoulder_roll = 0.15 - 0.12 * s
    right_shoulder_roll = -0.15 - 0.12 * s

    left_shoulder_pitch = 0.05 * s
    right_shoulder_pitch = -0.05 * s

    set_joint(model, data, "waist_roll_joint", waist_roll)
    set_joint(model, data, "waist_yaw_joint", waist_yaw)

    set_joint(model, data, "left_shoulder_roll_joint", left_shoulder_roll)
    set_joint(model, data, "right_shoulder_roll_joint", right_shoulder_roll)

    set_joint(model, data, "left_shoulder_pitch_joint", left_shoulder_pitch)
    set_joint(model, data, "right_shoulder_pitch_joint", right_shoulder_pitch)

    set_joint(model, data, "left_elbow_joint", 0.30)
    set_joint(model, data, "right_elbow_joint", 0.30)

def main():
    print(f"Loading: {XML_PATH}")

    if not XML_PATH.exists():
        raise FileNotFoundError(f"Soubor modelu nenalezen: {XML_PATH}")

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    print("Model loaded OK")
    print("Lekce: samostatné přenášení váhy bez zvedání nohou.")
    print("Jde o kinematickou demonstraci, ne o fyzikálně stabilizovanou rovnováhu.")
    print("Ukončení: zavři okno MuJoCo.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()

        while viewer.is_running():
            t = time.time() - start

            prepare_standing_pose(model, data)
            apply_weight_shift_only(model, data, t)

            mujoco.mj_forward(model, data)
            viewer.sync()

            time.sleep(0.01)


if __name__ == "__main__":
    main()