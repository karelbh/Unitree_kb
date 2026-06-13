import time
import math
import mujoco
import mujoco.viewer

from unitree_paths import load_g1_model


# Bezpečné parametry pro kinematický test
BASE_Z = 0.78
SHIFT_AMPLITUDE = 0.06      # 6 cm boční posun pánve
SHIFT_FREQUENCY = 0.15      # pomalý pohyb
FOOT_WARN_LIMIT = 0.003     # 3 mm povolený pohyb sledovaného bodu chodidla


def joint_qpos_addr(model, joint_name):
    jid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name
    )
    if jid < 0:
        raise RuntimeError(f"Kloub nenalezen: {joint_name}")
    return int(model.jnt_qposadr[jid])


def body_id(model, body_name):
    bid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        body_name
    )
    if bid < 0:
        raise RuntimeError(f"Body nenalezeno: {body_name}")
    return bid


def set_joint(model, data, joint_name, value):
    addr = joint_qpos_addr(model, joint_name)
    data.qpos[addr] = value


def prepare_base_pose(model, data):
    mujoco.mj_resetData(model, data)

    # Free joint základny: x, y, z, qw, qx, qy, qz
    data.qpos[0] = 0.0
    data.qpos[1] = 0.0
    data.qpos[2] = BASE_Z
    data.qpos[3] = 1.0
    data.qpos[4] = 0.0
    data.qpos[5] = 0.0
    data.qpos[6] = 0.0

    # Základní mírně pokrčený postoj
    set_joint(model, data, "left_hip_pitch_joint", -0.12)
    set_joint(model, data, "right_hip_pitch_joint", -0.12)

    set_joint(model, data, "left_knee_joint", 0.25)
    set_joint(model, data, "right_knee_joint", 0.25)

    set_joint(model, data, "left_ankle_pitch_joint", -0.12)
    set_joint(model, data, "right_ankle_pitch_joint", -0.12)

    set_joint(model, data, "waist_yaw_joint", 0.0)
    set_joint(model, data, "waist_roll_joint", 0.0)
    set_joint(model, data, "waist_pitch_joint", 0.0)


def apply_weight_shift(model, data, s):
    """
    Kinematický boční posun pánve.
    Zatím nepouštíme dynamiku přes mj_step.
    """

    pelvis_shift = SHIFT_AMPLITUDE * s

    # Posun základny / pánve do strany
    data.qpos[1] = pelvis_shift

    # Malá kompenzace nohou
    set_joint(model, data, "left_hip_roll_joint", -0.10 * s)
    set_joint(model, data, "right_hip_roll_joint", -0.10 * s)

    set_joint(model, data, "left_ankle_roll_joint", 0.08 * s)
    set_joint(model, data, "right_ankle_roll_joint", 0.08 * s)

    # Malá protiváha rukama — jen vizuální / mechanická intuice
    set_joint(model, data, "left_shoulder_roll_joint", 0.10 * s)
    set_joint(model, data, "right_shoulder_roll_joint", 0.10 * s)


def main():
    model = load_g1_model()
    data = mujoco.MjData(model)

    pelvis_id = body_id(model, "pelvis")
    torso_id = body_id(model, "torso_link")
    left_foot_id = body_id(model, "left_ankle_roll_link")
    right_foot_id = body_id(model, "right_ankle_roll_link")

    print("Model loaded OK")
    print("G1 weight shift monitor")
    print("Cíl: pánev/trup se posouvá do stran, sledované body chodidel zůstávají téměř fixované.")
    print("Zavři viewer pro konec.")
    print()

    prepare_base_pose(model, data)
    mujoco.mj_forward(model, data)

    left0 = data.xpos[left_foot_id].copy()
    right0 = data.xpos[right_foot_id].copy()

    last_print = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()

        while viewer.is_running():
            t = time.time() - start
            s = math.sin(2.0 * math.pi * SHIFT_FREQUENCY * t)

            prepare_base_pose(model, data)
            apply_weight_shift(model, data, s)

            mujoco.mj_forward(model, data)
            viewer.sync()

            if t - last_print > 1.0:
                last_print = t

                pelvis = data.xpos[pelvis_id]
                torso = data.xpos[torso_id]
                left = data.xpos[left_foot_id]
                right = data.xpos[right_foot_id]

                left_move = left - left0
                right_move = right - right0

                left_dy = left_move[1]
                right_dy = right_move[1]

                print(f"t = {t:6.2f} s")
                print(f"pelvis y       = {pelvis[1]: .4f}")
                print(f"torso  y       = {torso[1]: .4f}")
                print(f"left foot  dy  = {left_dy: .4f}")
                print(f"right foot dy  = {right_dy: .4f}")

                if abs(left_dy) > FOOT_WARN_LIMIT or abs(right_dy) > FOOT_WARN_LIMIT:
                    print("WARNING: foot movement above limit")

                print()

            time.sleep(0.01)


if __name__ == "__main__":
    main()