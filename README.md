<div align="center">

# ✋ → 🦾

### Move a robot arm with your bare hand

*A webcam, a hand, and a simulated SO-101 arm. No hardware, no gloves.*

`MediaPipe` · `MuJoCo` · `Python`

</div>

---

## What it does

Your webcam sees your hand. The simulated arm copies it.

```mermaid
flowchart LR
    A[📷 Webcam] --> B[MediaPipe<br/>21 hand points]
    B --> C[Clean up<br/>smooth · clamp · reject glitches]
    C --> D[Map to joints]
    D --> E[🦾 SO-101<br/>in MuJoCo]
```

| Your hand | The arm |
|---|---|
| Move left / right | Shoulder pans |
| Move up / down | Shoulder lifts |
| Move closer / farther from the camera | Elbow bends |
| Tilt your hand | Wrist rolls |
| Pinch thumb and index | Gripper closes |
| *(nothing, it's automatic)* | Wrist flex keeps the gripper level |
| Hand leaves the frame | Arm holds ~0.5 s, then returns to rest |

---

## Run it

You need Python 3.10+, a webcam, and the SO-101 model files.

**1. Get the arm model** (not included here, it belongs to its authors):

```bash
git clone https://github.com/TheRobotStudio/SO-ARM100.git
```

Clone it **next to** this folder, so the two sit side by side:

```
Stuff/
├── SO-ARM100/
└── [this-repo]/
```

**2. Add the cube scene.** Copy `assets/scene_cube.xml` into `SO-ARM100/Simulation/SO101/`.

**3. Set up Python:**

```bash
cd [this-repo]
python -m venv .venv
.venv\Scripts\activate
python -m pip install -r requirements.txt
```

**4. Download the hand model** (about 7 MB):

```powershell
Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task" -OutFile hand_landmarker.task
```

**5. Go:**

```bash
python handtoarm.py
```

Press `q` in the camera window to quit. Good light on your hand makes a big difference.

---

## Numbers measured on my own hand

I measured these instead of guessing. Yours will differ, so print them and adjust.

| Signal | Value | Setting |
|---|---|---|
| Pinch ratio, fingers touching | ~0.13 | `PINCH_CLOSED` |
| Pinch ratio, hand wide open | ~1.02 | `PINCH_OPEN` |
| Hand size, far from camera | ~0.15 | `SIZE_FAR` |
| Hand size, close to camera | ~0.45 | `SIZE_NEAR` |

The pinch is divided by the hand's own size, so it reads the same near or far.

---

## Knobs

| Setting | What it does |
|---|---|
| `ALPHA` | Smaller = smoother but slower to follow |
| `MAX_STEP` | A hard speed limit, so a twitch can't send the arm flying |
| `ZONE` | How much of the camera view covers the full arm range |
| `PAN_SIGN`, `LIFT_SIGN`, … | Flip to `-1` if a joint moves the wrong way |

---

## How far it got (honest status)

🟢 The arm follows my hand smoothly.
🟢 The gripper closes on a cube, with both jaws touching.
🟡 The cube lifts for about a second.
🔴 Then it falls.

<details>
<summary><b>Why it falls (my diagnosis)</b></summary>

<br>

A jaw-contact counter showed both jaws on the cube during the lift, and my pinch stayed closed. So it wasn't a weak gripper or my hand opening. In the frame before the drop, the cube is tilted about 30° while still gripped. The jaws hold it **off-centre**, near an edge, and a small cube held that way swings like a pendulum and slips out.

Ideas I haven't tested yet: grip at the cube's middle height, a bigger cube with more twist friction, a grip lock that stays closed until I clearly open my hand, and freezing the wrist while carrying.

</details>

<details>
<summary><b>What broke along the way</b></summary>

<br>

- **Arm moved opposite to my hand.** The model's "positive" isn't always "up" or "right". Fixed with a sign switch per joint.
- **Cube was out of reach.** I measured the arm's reach with 20,000 random poses (about 0.38 m, forward is +x) and moved the cube onto a table inside that zone.
- **Couldn't pick anything up.** The wrist was frozen, so the gripper lay flat when reaching. I tied the wrist to the shoulder and elbow to keep it level.
- **Arm flew around.** A speed limit per frame fixed it, at the cost of some lag.
- **Elbow range was too small.** My guess for hand size was off, so I measured it.

</details>

---

## Honest limits

- One webcam gives good left/right and up/down, **rough** depth, and **very rough** hand pitch. The elbow is the weakest control.
- Moving joint angles with your hand is hard for a person to steer. Inverse kinematics (hand position becomes a gripper target) is the natural next step.

---


<div align="center">

Built in simulation because hardware is expensive. Learning in public.

Arm model: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) · Hand tracking: [MediaPipe](https://ai.google.dev/edge/mediapipe)

</div>
