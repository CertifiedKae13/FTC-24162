# Limelight 3A SnapScript (Python) pipeline: BIOBUZZ FLOWER finder.
#
# Why this exists: the color pipeline + "Dual Target" grouping paired the flower's green posts with random
# dark floor-tile blobs (dark pixels have noisy hue and high saturation, so they passed the green threshold),
# so the target box and crosshair jumped onto the floor every few frames.
# This script only accepts tall, vertical, bright-enough green blobs, and prefers a matched PAIR of them
# (the two posts: similar height, tops level, sensible spacing). The returned contour is the hull of the pair,
# so the Limelight's normal tx / ty / ta point at the flower centre and the robot code needs no change.
#
# Install: Limelight web UI -> pipeline slot 1 (AutoConfig.LL_FLOWER_PIPELINE) -> Pipeline Type: Python ->
# paste this file -> Save. Tune the constants below while watching the stream (Show: Color).
#
# llpython output (read with LLResult.getPythonOutput()):
#   [0] found (1 pair, 0.5 single post, 0 none)  [1] centre x px  [2] centre y px  [3] post height px
#   [4] post spacing px (centre to centre, 0 if single)  [5] candidates seen  [6] 0  [7] 0

import cv2
import numpy as np

# HSV, OpenCV/Limelight scale: H 0-179, S 0-255, V 0-255. Measured on the recording: posts H 67-84,
# S 150-255, V 53-96 (far, dim gym). Floor tiles: V 20-37 -> V_MIN 48 cuts them out.
H_MIN, H_MAX = 62, 90
S_MIN, S_MAX = 120, 255
V_MIN, V_MAX = 48, 255        # posts' highlights reach V 240+ at high exposure; a lower cap broke the posts apart

MIN_POST_H_PX = 14            # ignore specks shorter than this
MIN_POST_ASPECT = 2.0         # rotated box long side / short side (posts are tall and thin)
MIN_FILL = 0.35               # contour area / rotated box area
MAX_TILT_DEG = 25             # posts are vertical; floor streaks and hive legs are not

PAIR_HEIGHT_RATIO = 1.8       # taller / shorter post
PAIR_TOP_TOL = 0.35           # |top1 - top2| <= this * height
PAIR_SPACING = (0.12, 1.6)    # centre spacing / height (far ~0.2, close ~0.8)
SINGLE_DOMINANCE = 3.0        # no pair: biggest blob must be this many times the next one to count


def post_candidates(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    posts = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if h < MIN_POST_H_PX or w == 0:
            continue
        # Shape is measured on the ROTATED rectangle: up close the camera sees the posts leaning, and an upright
        # box around a leaning post is wide and mostly empty (that rejected a real post before).
        (_, _), (rw, rh), ang = cv2.minAreaRect(c)
        long_side, short_side = max(rw, rh), max(1.0, min(rw, rh))
        if long_side < MIN_POST_ASPECT * short_side:
            continue
        area = cv2.contourArea(c)
        if area < MIN_FILL * long_side * short_side:
            continue
        # minAreaRect angle convention varies by OpenCV version; measure tilt of the long side from vertical
        long_is_w = rw > rh
        tilt = abs(((ang + (90 if long_is_w else 0)) + 90) % 180 - 90)
        if tilt > MAX_TILT_DEG:
            continue
        posts.append({"c": c, "x": x, "y": y, "w": w, "h": h, "cx": x + w / 2.0, "area": area})
    return posts


def best_pair(posts):
    best, best_score = None, 0.0
    for i in range(len(posts)):
        for j in range(i + 1, len(posts)):
            a, b = posts[i], posts[j]
            hmax, hmin = max(a["h"], b["h"]), min(a["h"], b["h"])
            if hmax > PAIR_HEIGHT_RATIO * hmin:
                continue
            if abs(a["y"] - b["y"]) > PAIR_TOP_TOL * hmax:
                continue
            spacing = abs(a["cx"] - b["cx"]) / hmax
            if not (PAIR_SPACING[0] <= spacing <= PAIR_SPACING[1]):
                continue
            score = a["area"] + b["area"]
            if score > best_score:
                best, best_score = (a, b), score
    return best


def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (H_MIN, S_MIN, V_MIN), (H_MAX, S_MAX, V_MAX))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # Join a post that is split by its rings / bands, without merging the two posts sideways.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 3), np.uint8))

    posts = post_candidates(mask)
    pair = best_pair(posts)
    llpython = [0, 0, 0, 0, 0, len(posts), 0, 0]
    target = np.array([[]])

    if pair is not None:
        a, b = pair
        target = cv2.convexHull(np.vstack([a["c"], b["c"]]))
        x, y, w, h = cv2.boundingRect(target)
        llpython = [1, x + w / 2.0, y + h / 2.0, max(a["h"], b["h"]), abs(a["cx"] - b["cx"]), len(posts), 0, 0]
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
        for p in pair:
            cv2.rectangle(image, (p["x"], p["y"]), (p["x"] + p["w"], p["y"] + p["h"]), (255, 0, 255), 1)
    elif posts:
        # No pair: one post hidden behind a hive leg, or both posts merged into one blob at long range.
        # Accept the biggest candidate only if it clearly dominates the rest. Weaker result (found = 0.5).
        posts.sort(key=lambda p: p["area"], reverse=True)
        p = posts[0]
        if len(posts) > 1 and p["area"] < SINGLE_DOMINANCE * posts[1]["area"]:
            p = None
    if pair is None and posts and p is not None:
        target = p["c"]
        llpython = [0.5, p["cx"], p["y"] + p["h"] / 2.0, p["h"], 0, 1, 0, 0]
        cv2.rectangle(image, (p["x"], p["y"]), (p["x"] + p["w"], p["y"] + p["h"]), (0, 165, 255), 2)

    for p in posts:
        cv2.rectangle(image, (p["x"], p["y"]), (p["x"] + p["w"], p["y"] + p["h"]), (80, 80, 80), 1)
    return target, image, llpython
