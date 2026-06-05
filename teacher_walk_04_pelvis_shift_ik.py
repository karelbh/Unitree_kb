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
    Základní kinematický postoj.
    """

    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    if model.nq >= 7:
        data.qpos[0] = 0.0
        data.qpos[1] = 0.0
        data.qpos[2] = BASE_Z

        data.qpos[3] = 1.0
        data.qpos[4] = 0.0
        data.qpos[5] = 0.0
        data.qpos[6] = 0.0

    # Nohy – základní mírně pokrčený postoj
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

    # Trup
    set_joint(model, data, "waist_yaw_joint", 0.0)
    set_joint(model, data, "waist_roll_joint", 0.0)
    set_joint(model, data, "waist_pitch_joint", 0.0)

    # Ruce
    set_joint(model, data, "left_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "left_shoulder_roll_joint", 0.15)
    set_joint(model, data, "left_elbow_joint", 0.30)

    set_joint(model, data, "right_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "right_shoulder_roll_joint", -0.15)
    set_joint(model, data, "right_elbow_joint", 0.30)


def apply_pelvis_shift_ik(model, data, t):
    """
    Experimentální aproximace přenášení váhy.

    Princip:
    - posuneme pánev/trup do strany pomocí floating base Y,
    - současně natočíme kyčle a kotníky tak, aby chodidla opticky
      zůstávala co nejvíce na místě.

    Není to plnohodnotný IK solver.
    Je to didaktický mezikrok.
    """

    frequency = 0.20
    phase = 2.0 * math.pi * frequency * t
    s = math.sin(phase)

    # Malý posun pánve vlevo/vpravo.
    # Záměrně malá hodnota.
    pelvis_y = 0.035 * s

    # Náklon trupu jako protiváha.
    waist_roll = -0.08 * s

    # Kompenzace nohou.
    # Tyto zisky bude možná nutné doladit podle vizuálního chování modelu.
    hip_gain = 1.20
    ankle_gain = 1.10

    left_hip_roll = 0.03 - hip_gain * pelvis_y
    right_hip_roll = -0.03 - hip_gain * pelvis_y

    left_ankle_roll = -0.03 + ankle_gain * pelvis_y
    right_ankle_roll = 0.03 + ankle_gain * pelvis_y

    # Mírná kompenzace kolen, aby postoj nepůsobil úplně tuhý.
    knee_comp = 0.04 * abs(s)

    left_knee = 0.28 + knee_comp
    right_knee = 0.28 + knee_comp

    # Ruce jako protiváha
    left_shoulder_roll = 0.15 + 0.10 * s
    right_shoulder_roll = -0.15 + 0.10 * s

    left_shoulder_pitch = 0.04 * s
    right_shoulder_pitch = -0.04 * s

    if model.nq >= 7:
        data.qpos[1] = pelvis_y

    set_joint(model, data, "waist_roll_joint", waist_roll)

    set_joint(model, data, "left_hip_roll_joint", left_hip_roll)
    set_joint(model, data, "right_hip_roll_joint", right_hip_roll)

    set_joint(model, data, "left_ankle_roll_joint", left_ankle_roll)
    set_joint(model, data, "right_ankle_roll_joint", right_ankle_roll)

    set_joint(model, data, "left_knee_joint", left_knee)
    set_joint(model, data, "right_knee_joint", right_knee)

    set_joint(model, data, "left_shoulder_roll_joint", left_shoulder_roll)
    set_joint(model, data, "right_shoulder_roll_joint", right_shoulder_roll)

    set_joint(model, data, "left_shoulder_pitch_joint", left_shoulder_pitch)
    set_joint(model, data, "right_shoulder_pitch_joint", right_shoulder_pitch)


def main():
    print(f"Loading: {XML_PATH}")

    if not XML_PATH.exists():
        raise FileNotFoundError(f"Soubor modelu nenalezen: {XML_PATH}")

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    print("Model loaded OK")
    print("Lekce 3B: experimentální posun pánve vůči chodidlům.")
    print("Cíl: chodidla co nejvíce na místě, pánev/trup do stran.")
    print("Není to ještě plnohodnotná inverzní kinematika.")
    print("Ukončení: zavři okno MuJoCo.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()

        while viewer.is_running():
            t = time.time() - start

            prepare_standing_pose(model, data)
            apply_pelvis_shift_ik(model, data, t)

            mujoco.mj_forward(model, data)
            viewer.sync()

            time.sleep(0.01)


if __name__ == "__main__":
    main()