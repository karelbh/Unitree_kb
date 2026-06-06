import time
import math

import mujoco
import mujoco.viewer

from unitree_paths import load_g1_model

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


def prepare_base_pose(model, data):
    """
    Kinematický základní postoj.
    Nejde o fyzikální stabilizaci, ale o přímé nastavení polohy robota.
    """

    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    # Floating base: poloha a orientace těla
    if model.nq >= 7:
        data.qpos[0] = 0.0
        data.qpos[1] = 0.0
        data.qpos[2] = BASE_Z

        # quaternion: vzpřímeně
        data.qpos[3] = 1.0
        data.qpos[4] = 0.0
        data.qpos[5] = 0.0
        data.qpos[6] = 0.0

    # Základní stojící poloha nohou
    set_joint(model, data, "left_hip_pitch_joint", -0.15)
    set_joint(model, data, "left_hip_roll_joint", 0.03)
    set_joint(model, data, "left_knee_joint", 0.30)
    set_joint(model, data, "left_ankle_pitch_joint", -0.15)
    set_joint(model, data, "left_ankle_roll_joint", -0.03)

    set_joint(model, data, "right_hip_pitch_joint", -0.15)
    set_joint(model, data, "right_hip_roll_joint", -0.03)
    set_joint(model, data, "right_knee_joint", 0.30)
    set_joint(model, data, "right_ankle_pitch_joint", -0.15)
    set_joint(model, data, "right_ankle_roll_joint", 0.03)

    # Trup
    set_joint(model, data, "waist_yaw_joint", 0.0)
    set_joint(model, data, "waist_roll_joint", 0.0)
    set_joint(model, data, "waist_pitch_joint", 0.0)

    # Ruce v klidu
    set_joint(model, data, "left_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "left_shoulder_roll_joint", 0.15)
    set_joint(model, data, "left_elbow_joint", 0.25)

    set_joint(model, data, "right_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "right_shoulder_roll_joint", -0.15)
    set_joint(model, data, "right_elbow_joint", 0.25)

    mujoco.mj_forward(model, data)


def apply_walking_motion(model, data, t):
    """
    Kinematická imitace chůze.
    Levá a pravá noha jsou v opačné fázi.
    """

    frequency = 0.55
    phase = 2.0 * math.pi * frequency * t

    left = math.sin(phase)
    right = math.sin(phase + math.pi)

    # Malý stranový náklon trupu - imitace přenášení váhy
    waist_roll = 0.05 * math.sin(phase)
    waist_yaw = 0.05 * math.sin(phase)

    set_joint(model, data, "waist_roll_joint", waist_roll)
    set_joint(model, data, "waist_yaw_joint", waist_yaw)

    # Levá noha
    left_hip_pitch = -0.15 + 0.25 * left
    left_knee = 0.35 + 0.20 * max(0.0, left)
    left_ankle_pitch = -0.18 - 0.12 * max(0.0, left)

    # Pravá noha
    right_hip_pitch = -0.15 + 0.25 * right
    right_knee = 0.35 + 0.20 * max(0.0, right)
    right_ankle_pitch = -0.18 - 0.12 * max(0.0, right)

    set_joint(model, data, "left_hip_pitch_joint", left_hip_pitch)
    set_joint(model, data, "left_knee_joint", left_knee)
    set_joint(model, data, "left_ankle_pitch_joint", left_ankle_pitch)

    set_joint(model, data, "right_hip_pitch_joint", right_hip_pitch)
    set_joint(model, data, "right_knee_joint", right_knee)
    set_joint(model, data, "right_ankle_pitch_joint", right_ankle_pitch)

    # Ruce proti nohám
    left_arm = -0.25 * left
    right_arm = -0.25 * right

    set_joint(model, data, "left_shoulder_pitch_joint", left_arm)
    set_joint(model, data, "right_shoulder_pitch_joint", right_arm)

    # Lokty mírně pokrčené
    set_joint(model, data, "left_elbow_joint", 0.35)
    set_joint(model, data, "right_elbow_joint", 0.35)


def main():
    model = load_g1_model()
    data = mujoco.MjData(model)

    print("Model loaded OK")
    print("Lekce 2: kinematická imitace chůze.")
    print("Nejde o dynamickou chůzi. Robot neřeší rovnováhu, jen ukazuje pohyb kloubů.")
    print("Ukončení: zavři okno MuJoCo.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()

        while viewer.is_running():
            t = time.time() - start

            prepare_base_pose(model, data)
            apply_walking_motion(model, data, t)

            mujoco.mj_forward(model, data)
            viewer.sync()

            time.sleep(0.01)


if __name__ == "__main__":
    main()