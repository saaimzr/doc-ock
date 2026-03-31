#!/usr/bin/env python3

import argparse
import time

import cv2
import numpy as np
import serial


def _build_homography_from_clicks(clicked_points, world_pts):
    image_pts = np.array(clicked_points, dtype=np.float32)
    return cv2.getPerspectiveTransform(image_pts, world_pts)


def _extract_target_from_largest_red_contour(frame, min_red_area):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_red_area:
        return None

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2
    cy = y + h // 2
    return cx, cy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM6", help="serial port, e.g. COM6")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--z", type=float, default=4.5, help="fixed grasp height (cm)")
    ap.add_argument("--send-period", type=float, default=0.20, help="send rate in seconds (default: 5 hz)")
    ap.add_argument("--min-red-area", type=float, default=500)
    ap.add_argument("--send-auto", action="store_true", help="send AUTO once after startup")
    ap.add_argument("--quiet", action="store_true", help="don't print sent uart payloads")
    args = ap.parse_args()

    clicked_points = []
    h = None

    def mouse_callback(event, x, y, flags, param):
        nonlocal clicked_points
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
            clicked_points.append([x, y])
            print(f"clicked pixel point: ({x}, {y})")

    cap = cv2.VideoCapture(0)
    cv2.namedWindow("calibration")
    cv2.setMouseCallback("calibration", mouse_callback)

    print("click the 4 corners of the workspace in this order:")
    print("1. top-left")
    print("2. top-right")
    print("3. bottom-right")
    print("4. bottom-left")

    # workspace units are whatever you want to send to the stm32.
    # if you already calibrated this to match the robot workspace (cm), keep this in cm.
    world_pts = np.array(
        [
            [0, 0],
            [25, 0],
            [25, 20],
            [0, 20],
        ],
        dtype=np.float32,
    )

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    time.sleep(2.0)
    print(f"opened serial port {args.port} @ {args.baud} baud")

    if args.send_auto:
        # stm32 starts in manual mode; auto runs the pick/place cycle.
        ser.write(b"AUTO\n")
        ser.flush()

    last_send_time = 0.0
    last_payload = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            display = frame.copy()
            for pt in clicked_points:
                cv2.circle(display, tuple(pt), 5, (0, 255, 0), -1)

            if len(clicked_points) == 4 and h is None:
                h = _build_homography_from_clicks(clicked_points, world_pts)
                print("perspective transform computed.")

            target = _extract_target_from_largest_red_contour(frame, args.min_red_area)
            if target is not None:
                cx, cy = target
                cv2.circle(display, (cx, cy), 6, (255, 0, 0), -1)
                cv2.putText(
                    display,
                    f"pixel: ({cx},{cy})",
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

                if h is not None:
                    pixel_pt = np.array([[[cx, cy]]], dtype=np.float32)
                    world_pt = cv2.perspectiveTransform(pixel_pt, h)
                    x, y = world_pt[0][0]
                    z = args.z

                    cv2.putText(
                        display,
                        f"world: ({x:.2f} cm, {y:.2f} cm, {z:.2f} cm)",
                        (cx + 10, cy + 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2,
                    )

                    payload = f"T {x:.2f} {y:.2f} {z:.2f}\n"
                    now = time.time()

                    if (now - last_send_time) >= args.send_period:
                        ser.write(payload.encode("ascii", errors="ignore"))
                        ser.flush()
                        last_send_time = now

                        if (payload != last_payload) and (not args.quiet):
                            print("sent:", payload.strip())
                            last_payload = payload

            cv2.imshow("calibration", display)
            key = cv2.waitKey(1)
            if key == 27:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()


if __name__ == "__main__":
    main()

