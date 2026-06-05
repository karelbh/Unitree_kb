import time
import math

import cv2
import numpy as np


WIDTH = 640
HEIGHT = 480


def create_virtual_camera_frame(t):
    """
    Vytvoří umělý obraz jako z kamery.
    V obrazu se pohybuje červený objekt.
    """

    # Černé pozadí
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    # Pohyb objektu po sinusové trajektorii
    x = int(WIDTH / 2 + 200 * math.sin(2.0 * math.pi * 0.15 * t))
    y = int(HEIGHT / 2 + 100 * math.sin(2.0 * math.pi * 0.10 * t))

    # Červený kruh
    cv2.circle(frame, (x, y), 35, (0, 0, 255), -1)

    # Text v obraze
    cv2.putText(
        frame,
        "Virtual camera - red object detection",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    return frame


def detect_red_object(frame):
    """
    Najde červený objekt v obraze.
    Vrací souřadnice středu objektu.
    """

    # Převod BGR → HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Červená barva je v HSV na začátku a konci rozsahu H.
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])

    lower_red_2 = np.array([160, 100, 100])
    upper_red_2 = np.array([179, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

    mask = mask1 + mask2

    # Najít kontury
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None, mask

    # Největší červený objekt
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 100:
        return None, mask

    # Střed objektu
    moments = cv2.moments(largest)

    if moments["m00"] == 0:
        return None, mask

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])

    return (cx, cy, area), mask


def main():
    print("Virtual camera started.")
    print("Press Q in the image window to quit.")

    start = time.time()

    while True:
        t = time.time() - start

        # Virtuální obraz
        frame = create_virtual_camera_frame(t)

        # Detekce červeného objektu
        result, mask = detect_red_object(frame)

        if result is not None:
            cx, cy, area = result

            # Vykreslení středu objektu
            cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

            text = f"Object: x={cx}, y={cy}, area={int(area)}"
            cv2.putText(
                frame,
                text,
                (20, HEIGHT - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            print(f"x={cx:3d}, y={cy:3d}, area={area:.0f}")

        else:
            cv2.putText(
                frame,
                "No red object detected",
                (20, HEIGHT - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow("Virtual camera", frame)
        cv2.imshow("Red mask", mask)

        key = cv2.waitKey(30) & 0xFF

        if key == ord("q") or key == ord("Q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()