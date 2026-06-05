import time
from pathlib import Path

import cv2
import numpy as np
import mujoco
import mujoco.viewer


def find_unitree_root():
    possible_roots = [
        Path(r"F:\Unitree"),  # doma
        Path(r"U:\Unitree"),  # práce
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

WIDTH = 640
HEIGHT = 480
BASE_Z = 0.78

mouse_x = WIDTH // 2
mouse_y = HEIGHT // 2


def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y

    if event == cv2.EVENT_MOUSEMOVE or event == cv2.EVENT_LBUTTONDOWN:
        mouse_x = x
        mouse_y = y


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


def prepare_pose(model, data):
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

    # Přibližná stojící poloha nohou
    set_joint(model, data, "left_hip_pitch_joint", -0.15)
    set_joint(model, data, "left_knee_joint", 0.30)
    set_joint(model, data, "left_ankle_pitch_joint", -0.15)

    set_joint(model, data, "right_hip_pitch_joint", -0.15)
    set_joint(model, data, "right_knee_joint", 0.30)
    set_joint(model, data, "right_ankle_pitch_joint", -0.15)

    # Základní poloha rukou
    set_joint(model, data, "left_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "left_shoulder_roll_joint", 0.15)
    set_joint(model, data, "left_elbow_joint", 0.30)

    set_joint(model, data, "right_shoulder_pitch_joint", 0.0)
    set_joint(model, data, "right_shoulder_roll_joint", -0.15)
    set_joint(model, data, "right_elbow_joint", 0.30)

    mujoco.mj_forward(model, data)


def create_virtual_camera_frame():
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    # Středové osy obrazu
    cv2.line(frame, (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), (70, 70, 70), 1)
    cv2.line(frame, (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), (70, 70, 70), 1)

    # Červený objekt ovládaný myší
    cv2.circle(frame, (mouse_x, mouse_y), 35, (0, 0, 255), -1)

    cv2.putText(
        frame,
        "Move mouse: red object controls robot",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        "Q = quit",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (180, 180, 180),
        2
    )

    return frame


def detect_red_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])

    lower_red_2 = np.array([160, 100, 100])
    upper_red_2 = np.array([179, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

    mask = mask1 + mask2

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 100:
        return None, mask

    moments = cv2.moments(largest)

    if moments["m00"] == 0:
        return None, mask

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])

    return (cx, cy, area), mask


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def main():
    print(f"Loading: {XML_PATH}")

    if not XML_PATH.exists():
        raise FileNotFoundError(f"Soubor modelu nenalezen: {XML_PATH}")

    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    print("Model loaded OK")
    print("Myší ovládej červený objekt v okně Virtual camera.")
    print("Robot bude reagovat trupem a levou rukou.")
    print("Pohyb je vyhlazený filtrem alpha.")
    print("Ukončení: Q v okně kamery nebo zavření MuJoCo.")

    cv2.namedWindow("Virtual camera")
    cv2.setMouseCallback("Virtual camera", mouse_callback)

    # Vyhlazené hodnoty kloubů
    smooth_waist_yaw = 0.0
    smooth_left_shoulder_pitch = 0.0

    # Čím menší alpha, tím plynulejší a pomalejší pohyb.
    alpha = 0.10

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            frame = create_virtual_camera_frame()
            result, mask = detect_red_object(frame)

            prepare_pose(model, data)

            if result is not None:
                cx, cy, area = result

                # Normalizovaná odchylka od středu obrazu
                error_x = (cx - WIDTH / 2) / (WIDTH / 2)
                error_y = (cy - HEIGHT / 2) / (HEIGHT / 2)

                error_x = clamp(error_x, -1.0, 1.0)
                error_y = clamp(error_y, -1.0, 1.0)

                # Požadované hodnoty kloubů
                target_waist_yaw = -0.45 * error_x
                target_left_shoulder_pitch = 0.45 * error_y
                left_elbow = -0.45

                # Vyhlazení pohybu
                smooth_waist_yaw = (
                    (1.0 - alpha) * smooth_waist_yaw
                    + alpha * target_waist_yaw
                )

                smooth_left_shoulder_pitch = (
                    (1.0 - alpha) * smooth_left_shoulder_pitch
                    + alpha * target_left_shoulder_pitch
                )

                # Nastavení kloubů robota
                set_joint(model, data, "waist_yaw_joint", smooth_waist_yaw)
                set_joint(model, data, "left_shoulder_pitch_joint", smooth_left_shoulder_pitch)
                set_joint(model, data, "left_elbow_joint", left_elbow)

                # Zobrazení detekce
                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
                cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

                text = (
                    f"x={cx}, y={cy}, "
                    f"ex={error_x:+.2f}, ey={error_y:+.2f}, "
                    f"waist={smooth_waist_yaw:+.2f}, "
                    f"shoulder={smooth_left_shoulder_pitch:+.2f}"
                )

                cv2.putText(
                    frame,
                    text,
                    (20, HEIGHT - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2
                )

                print(
                    f"x={cx:3d}, y={cy:3d}, "
                    f"ex={error_x:+.2f}, ey={error_y:+.2f}, "
                    f"target_waist={target_waist_yaw:+.2f}, "
                    f"smooth_waist={smooth_waist_yaw:+.2f}, "
                    f"smooth_shoulder={smooth_left_shoulder_pitch:+.2f}"
                )

            mujoco.mj_forward(model, data)
            viewer.sync()

            cv2.imshow("Virtual camera", frame)
            cv2.imshow("Red mask", mask)

            key = cv2.waitKey(20) & 0xFF

            if key == ord("q") or key == ord("Q"):
                break

            time.sleep(0.01)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()