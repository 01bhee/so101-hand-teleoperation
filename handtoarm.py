import time
import cv2
import numpy as np
import mujoco
import mujoco.viewer
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

import os
HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "SO-ARM100", "Simulation", "SO101", "scene_cube.xml")
USE_FRACTION = 0.8
ZONE = (0.1, 0.9)
MAX_JUMP = 0.25
OUTLIER_LIMIT = 5
HOLD_FRAMES = 15

PAN_SIGN = -1
LIFT_SIGN = -1
GRIP_SIGN = 1
ELBOW_SIGN = 1
ROLL_SIGN = 1

ALPHA = 0.08
ELBOW_ALPHA = 0.05
ROLL_ALPHA = 0.10
GRIP_ALPHA = 0.30
MAX_STEP = 0.03

PINCH_CLOSED = 0.15
PINCH_OPEN = 1.00
SIZE_FAR = 0.15
SIZE_NEAR = 0.40
ROLL_RANGE = 0.8

WRIST_K = -1.0
WRIST_OFFSET = 0.0

model = mujoco.MjModel.from_xml_path(SCENE)
data = mujoco.MjData(model)
lo = model.actuator_ctrlrange[:, 0]
hi = model.actuator_ctrlrange[:, 1]
mid = (lo + hi) / 2
half = (hi - lo) / 2

PAN, LIFT, ELBOW, WRIST_FLEX, WRIST_ROLL, GRIP = 0, 1, 2, 3, 4, 5

def to_joint(i, v, sign):
    v = np.clip(v, 0.0, 1.0)
    return mid[i] + sign * (v * 2 - 1) * USE_FRACTION * half[i]

def zone(v):
    return np.clip((v - ZONE[0]) / (ZONE[1] - ZONE[0]), 0.0, 1.0)

def dist(a, b):
    return float(np.hypot(a.x - b.x, a.y - b.y))

def step_toward(current, target, alpha, max_step):
    delta = alpha * (target - current)
    return current + np.clip(delta, -max_step, max_step)

options = vision.HandLandmarkerOptions(
    base_options=mp_python.BaseOptions(model_asset_path=os.path.join(HERE, "hand_landmarker.task")),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=1,
)

cap = cv2.VideoCapture(0)
start = time.time()
last_time = start

accepted = np.array([0.5, 0.5])
smooth = accepted.copy()
grip_target = 0.5
grip_smooth = 0.5
extra_target = np.array([0.5, 0.5])
extra_smooth = extra_target.copy()
extra_alpha = np.array([ELBOW_ALPHA, ROLL_ALPHA])
outlier_count = 0
missed = 0

data.ctrl[:] = mid

cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
jaw_bodies = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
              for n in ("gripper", "moving_jaw_so101_v1")}

with vision.HandLandmarker.create_from_options(options) as landmarker, \
     mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        ts = int((time.time() - start) * 1000)
        result = landmarker.detect_for_video(mp_image, ts)

        if result.hand_landmarks:
            missed = 0
            hand = result.hand_landmarks[0]
            tip = hand[8]
            raw = np.array([zone(tip.x), zone(1.0 - tip.y)])

            if np.max(np.abs(raw - accepted)) > MAX_JUMP and outlier_count < OUTLIER_LIMIT:
                outlier_count += 1
            else:
                outlier_count = 0
                accepted = raw

            size = dist(hand[0], hand[9])
            ratio = dist(hand[4], hand[8]) / max(size, 1e-6)
            grip_target = float(np.clip(
                (ratio - PINCH_CLOSED) / (PINCH_OPEN - PINCH_CLOSED), 0.0, 1.0))

            elbow_v = np.clip((size - SIZE_FAR) / (SIZE_NEAR - SIZE_FAR), 0.0, 1.0)

            dx = hand[9].x - hand[0].x
            dy = hand[9].y - hand[0].y
            angle = np.arctan2(dx, -dy)
            roll_v = np.clip((angle / ROLL_RANGE + 1) / 2, 0.0, 1.0)

            extra_target = np.array([elbow_v, roll_v])
            print(f"pinch={ratio:.2f}  size={size:.2f}  angle={angle:.2f}")

            h, w, _ = frame.shape
            cv2.circle(frame, (int(tip.x * w), int(tip.y * h)), 8, (0, 255, 0), -1)
            cv2.circle(frame, (int(hand[4].x * w), int(hand[4].y * h)), 8, (255, 0, 0), -1)
        else:
            missed += 1
            if missed > HOLD_FRAMES:
                accepted = np.array([0.5, 0.5])
                grip_target = 0.5
                extra_target = np.array([0.5, 0.5])

        smooth = step_toward(smooth, accepted, ALPHA, MAX_STEP)
        grip_smooth = step_toward(grip_smooth, grip_target, GRIP_ALPHA, 0.08)
        extra_smooth = step_toward(extra_smooth, extra_target, extra_alpha, MAX_STEP)

        lift_t = to_joint(LIFT, smooth[1], LIFT_SIGN)
        elbow_t = to_joint(ELBOW, extra_smooth[0], ELBOW_SIGN)

        data.ctrl[PAN] = to_joint(PAN, smooth[0], PAN_SIGN)
        data.ctrl[LIFT] = lift_t
        data.ctrl[ELBOW] = elbow_t
        data.ctrl[WRIST_FLEX] = np.clip(WRIST_OFFSET + WRIST_K * (lift_t + elbow_t),
                                        lo[WRIST_FLEX], hi[WRIST_FLEX])
        data.ctrl[WRIST_ROLL] = to_joint(WRIST_ROLL, extra_smooth[1], ROLL_SIGN)
        data.ctrl[GRIP] = to_joint(GRIP, grip_smooth, GRIP_SIGN)

        now = time.time()
        steps = max(1, int((now - last_time) / model.opt.timestep))
        last_time = now
        for _ in range(steps):
            mujoco.mj_step(model, data)
        viewer.sync()

        touching = set()
        for i in range(data.ncon):
            c = data.contact[i]
            if cube_geom in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == cube_geom else c.geom1
                b = model.geom_bodyid[other]
                if b in jaw_bodies:
                    touching.add(b)
        jaws_touching = len(touching)

        status = "tracking" if missed == 0 else f"hand lost ({missed})"
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"jaws touching: {jaws_touching}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 255), 2)
        cv2.imshow("hand", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("w"):
            WRIST_OFFSET += 0.1
        elif key == ord("s"):
            WRIST_OFFSET -= 0.1

cap.release()
cv2.destroyAllWindows()
