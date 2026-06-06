import time
import numpy as np
import mujoco
import mujoco.viewer

from unitree_paths import load_g1_model

model = load_g1_model()
data = mujoco.MjData(model)

# U tvého modelu se klouby jmenují s příponou _joint
LEFT_ARM_JOINTS = {
    "shoulder_pitch": "left_shoulder_pitch_joint",
    "shoulder_roll": "left_shoulder_roll_joint",
    "shoulder_yaw": "left_shoulder_yaw_joint",
    "elbow": "left_elbow_joint",
    "wrist_roll": "left_wrist_roll_joint",
    "wrist_pitch": "left_wrist_pitch_joint",
    "wrist_yaw": "left_wrist_yaw_joint",
}


def joint_qpos_index(model, joint_name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

    if jid < 0:
        print(f"\nCHYBA: Kloub nenalezen: {joint_name}")
        print("\nDostupné klouby v modelu:")
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(i, name)
        raise ValueError(f"Kloub nenalezen: {joint_name}")

    return model.jnt_qposadr[jid], jid


def get_joint_angle(model, data, joint_name):
    qadr, _ = joint_qpos_index(model, joint_name)
    return float(data.qpos[qadr])


def set_joint_angle(model, data, joint_name, angle):
    qadr, jid = joint_qpos_index(model, joint_name)

    # Dodržení limitů kloubu
    if model.jnt_limited[jid]:
        lo, hi = model.jnt_range[jid]
        angle = float(np.clip(angle, lo, hi))

    data.qpos[qadr] = angle


def print_help():
    print()
    print("=== OVLÁDÁNÍ LEVÉ RUKY G1 ===")
    print("Klikni do okna MuJoCo vieweru.")
    print()
    print("I / K  = rameno dopředu / dozadu     shoulder_pitch")
    print("J / L  = rameno do strany            shoulder_roll")
    print("U / O  = rotace ramene               shoulder_yaw")
    print("W / S  = loket                       elbow")
    print("A / D  = zápěstí pitch               wrist_pitch")
    print("Q / E  = zápěstí yaw                 wrist_yaw")
    print("Z / C  = zápěstí roll                wrist_roll")
    print("R      = reset ruky")
    print("H      = nápověda")
    print("ESC    = konec")
    print()


print("Loading:", MODEL_PATH)

if not MODEL_PATH.exists():
    raise FileNotFoundError(MODEL_PATH)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print("Model loaded OK")
print("Joints:", model.njnt)
print("Actuators:", model.nu)

# Bezpečný kinematický režim:
# - nepoužíváme mj_step()
# - nepoužíváme dynamiku
# - robot nepadá
# - robot neletí nahoru
model.opt.gravity[:] = [0.0, 0.0, 0.0]

# Výchozí poloha
if model.nkey > 0:
    print("Používám keyframe 0")
    data.qpos[:] = model.key_qpos[0]
else:
    print("Používám qpos0")
    data.qpos[:] = model.qpos0

data.qvel[:] = 0.0

# Fixace těla ve vzpřímené poloze.
# qpos[0:3] = poloha těla
# qpos[3:7] = kvaternion orientace
if model.nq >= 7:
    data.qpos[0] = 0.0
    data.qpos[1] = 0.0
    data.qpos[2] = 0.75
    data.qpos[3] = 1.0
    data.qpos[4] = 0.0
    data.qpos[5] = 0.0
    data.qpos[6] = 0.0

mujoco.mj_forward(model, data)

# Počáteční úhly levé ruky
home_angles = {
    key: get_joint_angle(model, data, joint_name)
    for key, joint_name in LEFT_ARM_JOINTS.items()
}

angles = home_angles.copy()

step = 0.08
running = True


def show_angles():
    print("Úhly:", {k: round(v, 2) for k, v in angles.items()})


def key_callback(keycode):
    global running, angles

    # ESC ve GLFW bývá 256
    if keycode == 256:
        running = False
        return

    ch = chr(keycode).lower() if 0 <= keycode < 256 else ""

    if ch == "h":
        print_help()

    elif ch == "i":
        angles["shoulder_pitch"] -= step
    elif ch == "k":
        angles["shoulder_pitch"] += step

    elif ch == "j":
        angles["shoulder_roll"] += step
    elif ch == "l":
        angles["shoulder_roll"] -= step

    elif ch == "u":
        angles["shoulder_yaw"] += step
    elif ch == "o":
        angles["shoulder_yaw"] -= step

    elif ch == "w":
        angles["elbow"] += step
    elif ch == "s":
        angles["elbow"] -= step

    elif ch == "a":
        angles["wrist_pitch"] += step
    elif ch == "d":
        angles["wrist_pitch"] -= step

    elif ch == "q":
        angles["wrist_yaw"] += step
    elif ch == "e":
        angles["wrist_yaw"] -= step

    elif ch == "z":
        angles["wrist_roll"] += step
    elif ch == "c":
        angles["wrist_roll"] -= step

    elif ch == "r":
        angles = home_angles.copy()

    else:
        return

    show_angles()


print_help()

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    print("Viewer spuštěn.")
    print("Klikni do okna vieweru a ovládej ruku klávesami.")
    print("Doporučený první test: I I I W W J J")

    while viewer.is_running() and running:
        # Držení těla na místě
        if model.nq >= 7:
            data.qpos[0] = 0.0
            data.qpos[1] = 0.0
            data.qpos[2] = 0.75
            data.qpos[3] = 1.0
            data.qpos[4] = 0.0
            data.qpos[5] = 0.0
            data.qpos[6] = 0.0

        # Nastavení úhlů kloubů levé ruky
        for key, joint_name in LEFT_ARM_JOINTS.items():
            set_joint_angle(model, data, joint_name, angles[key])

        # Žádná dynamika, pouze přepočet geometrie
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        viewer.sync()
        time.sleep(0.01)

print("Konec.")