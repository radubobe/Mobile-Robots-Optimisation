"""
Automated test campaign: runs (road × width × model) combinations in headless
Webots and collects oracle verdicts into a CSV.

    python batch_runner.py --smoke        # 4 tests, sanity check
    python batch_runner.py                # full campaign
    python batch_runner.py --roads test.0001 test.0002 --widths 0.20

Resumes from where it left off; already-completed rows are skipped.
"""

import argparse
import csv
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import time

PROJECT_ROOT = r"C:\ExtendedController"
WEBOTS_EXE = r"C:\Webots\msys64\mingw64\bin\webotsw.exe"

CONTROLLER_DIR = "enps_controller"
SETTINGS_REL = os.path.join("models", "1", "Settings.txt")
WEIGHTS_FILE = "Weights"

ROAD_Z = 1.0
EPUCK_BODY_RADIUS = 0.037
ROBOT_DROP = 0.0        # keep at 0; larger values cause the robot to fall at start

DEFAULT_TIMEOUT = 900   # wall-clock safety net per test

ROADS = ["validation.%04d" % i for i in range(1, 51)]
WIDTHS = [0.12, 0.15, 0.20, 0.25]
MODELS = ["Model 1", "Model 2", "Model 3"]

CSV_COLUMNS = [
    "road", "width", "model", "status", "failed", "failure_mode",
    "steps", "simulated_seconds", "road_length_m", "progress_m",
    "max_lateral_m", "road_width_m", "oob_threshold_m",
    "rotating_steps", "straight_steps",
    "f1_margin_to_oob", "f2_progress_fraction", "f3_neg_delta_v",
    "wall_seconds", "run_id",
]



WORLD_TEMPLATE = """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"
EXTERNPROTO "../protos/RoadFromPoints.proto"

WorldInfo {{
}}
Viewpoint {{
  orientation -0.5773502691896257 0.5773502691896257 0.5773502691896257 2.0943245915045035
  position {vx} {vy} 5.004471997262138
  follow "e-puck"
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
Floor {{
  translation 0 0 -0.02
  size 100 100
  tileSize 1 1
  appearance PBRAppearance {{
    baseColorMap ImageTexture {{
      url [
        "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/vehicles/worlds/textures/ground_grass.jpg"
      ]
    }}
    roughness 1
    metalness 0
  }}
}}
RoadFromPoints {{
  translation 0 0 1
  fileName "{road}"
  width {width}
  rightBorderBoundingObject FALSE
  leftBorderBoundingObject FALSE
}}
DEF E-puck E-puck {{
  translation {x} {y} {z}
  rotation 0 0 1 {heading}
  controller "{controller}"
  controllerArgs [ "{road}" "{width}" ]
  turretSlot [
    GPS {{
    }}
  ]
}}
Robot {{
  supervisor TRUE
}}
"""


def read_spine(path):
    with open(path, "r") as handle:
        raw = handle.read()
    points = []
    for line in raw.replace("\r\n", "\n").strip().split("\n"):
        line = line.strip()
        if line:
            parts = line.split()
            points.append((float(parts[0]), float(parts[1])))
    if len(points) < 2:
        raise ValueError("Spine prea scurt: %s" % path)
    return points


def start_pose(spine, offset=None):
    # Robot is offset straight along the initial heading, not along the polyline.
    # A sub-mm deviation here changes trajectories — do not modify without
    # re-validating against manual runs.
    if offset is None:
        offset = EPUCK_BODY_RADIUS

    heading = math.atan2(spine[1][1] - spine[0][1], spine[1][0] - spine[0][0])
    x = spine[0][0] + offset * math.cos(heading)
    y = spine[0][1] + offset * math.sin(heading)
    return x, y, heading


def _fmt(value, decimals=6):
    # Truncate (not round) to 6 decimals, matching Webots scene-tree output.
    # A 1e-6 difference is enough to flip a quantised sensor and change the trajectory.
    scale = 10 ** decimals
    truncated = math.trunc(value * scale) / scale
    if truncated == int(truncated):
        return str(int(truncated))
    return ("%.*f" % (decimals, truncated)).rstrip("0").rstrip(".")


def write_world(path, road, width, spine, controller=CONTROLLER_DIR):
    x, y, heading = start_pose(spine)
    xs = [p[0] for p in spine]
    ys = [p[1] for p in spine]
    content = WORLD_TEMPLATE.format(
        road=road, width=width,
        x=_fmt(x), y=_fmt(y), z=_fmt(ROAD_Z + ROBOT_DROP), heading=_fmt(heading),
        vx=_fmt(sum(xs) / len(xs)), vy=_fmt(sum(ys) / len(ys)),
        controller=controller)
    with open(path, "w", newline="\r\n") as handle:
        handle.write(content)


def write_settings(root, model_name):
    path = os.path.join(root, SETTINGS_REL)
    with open(path, "w") as handle:
        handle.write("modelFileName=%s\n" % model_name)
        handle.write("weightFileName=%s\n" % WEIGHTS_FILE)


def newest_verdict(controller_path, after_time):
    candidates = []
    for pattern in ("verdict_*.json",):
        for path in glob.glob(os.path.join(controller_path, pattern)):
            if os.path.getmtime(path) >= after_time - 1:
                candidates.append(path)
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def cleanup_controller_outputs(controller_path, keep_traces):
    patterns = ["verdict_*.json"]
    if not keep_traces:
        patterns += ["robot_path_*.txt", "pep_*.csv"]
    for pattern in patterns:
        for path in glob.glob(os.path.join(controller_path, pattern)):
            try:
                os.remove(path)
            except OSError:
                pass


def archive_traces(controller_path, archive_dir, run_id):
    os.makedirs(archive_dir, exist_ok=True)
    for pattern, prefix in (("robot_path_*.txt", "path"), ("pep_*.csv", "pep")):
        files = glob.glob(os.path.join(controller_path, pattern))
        if not files:
            continue
        newest = max(files, key=os.path.getmtime)
        ext = os.path.splitext(newest)[1]
        shutil.move(newest, os.path.join(archive_dir,
                                         "%s_%s%s" % (prefix, run_id, ext)))
        for other in files:
            if os.path.exists(other):
                try:
                    os.remove(other)
                except OSError:
                    pass


def _webots_still_running(exe_name="webots"):
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq %s*" % exe_name],
                             capture_output=True, text=True, timeout=15).stdout
        return exe_name.lower() in out.lower()
    except (OSError, subprocess.SubprocessError):
        return False


def _kill_stray_webots():
    for image in ("webotsw.exe", "webots.exe", "webots-bin.exe"):
        try:
            subprocess.run(["taskkill", "/F", "/IM", image],
                           capture_output=True, timeout=15)
        except (OSError, subprocess.SubprocessError):
            pass


def wait_for_clean_slate(timeout=30):
    # A leftover Webots instance causes the next one to start in an abnormal
    # state and produce different results for the same configuration — only
    # observable in batch runs, not in isolation.
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not _webots_still_running():
            return True
        time.sleep(0.5)
    _kill_stray_webots()
    time.sleep(2.0)
    return not _webots_still_running()


def run_one(root, webots, road, width, model, keep_traces, timeout):
    run_id = "%s_w%s_%s" % (road, str(width).replace(".", ""),
                            model.replace(" ", ""))
    controller_path = os.path.join(root, "controllers", CONTROLLER_DIR)
    world_path = os.path.join(root, "worlds", "_batch_current.wbt")

    spine = read_spine(os.path.join(root, "data", road + ".txt"))
    write_world(world_path, road, width, spine)
    write_settings(root, model)
    cleanup_controller_outputs(controller_path, keep_traces=False)

    if not wait_for_clean_slate():
        sys.stderr.write("  [WARNING] a Webots instance is still running; "
                         "this test result may be unreliable\n")

    started = time.time()
    command = [webots, "--batch", "--mode=fast", "--no-rendering",
               "--minimize", "--stdout", "--stderr", world_path]

    # Webots does not close when the controller loop ends, so we launch it as a
    # subprocess and terminate it as soon as the oracle writes the verdict.
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    verdict_path = None
    deadline = started + timeout
    while time.time() < deadline:
        verdict_path = newest_verdict(controller_path, started)
        if verdict_path is not None:
            time.sleep(0.3)  # let the file write finish
            break
        if process.poll() is not None:  # Webots exited without writing a verdict
            break
        time.sleep(0.25)

    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    wall = time.time() - started

    wait_for_clean_slate()  # child processes may outlive the main Webots window

    if verdict_path is None:
        verdict_path = newest_verdict(controller_path, started)
    if verdict_path is None:
        sys.stderr.write("  [no verdict] %s\n" % run_id)
        return None
    with open(verdict_path, "r") as handle:
        verdict = json.load(handle)

    if keep_traces:
        archive_traces(controller_path, os.path.join(root, "batch_traces"), run_id)
    cleanup_controller_outputs(controller_path, keep_traces=False)

    verdict.update({"road": road, "width": width, "model": model,
                    "wall_seconds": round(wall, 1), "run_id": run_id})
    return verdict


def load_done(csv_path):
    if not os.path.exists(csv_path):
        return set()
    done = set()
    with open(csv_path, "r", newline="") as handle:
        for row in csv.DictReader(handle):
            done.add((row["road"], row["width"], row["model"]))
    return done


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=PROJECT_ROOT)
    parser.add_argument("--webots", default=WEBOTS_EXE)
    parser.add_argument("--roads", nargs="*", default=None)
    parser.add_argument("--widths", nargs="*", type=float, default=None)
    parser.add_argument("--models", nargs="*", default=None)
    parser.add_argument("--out", default="batch_results.csv")
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT,
                        help="wall-clock seconds per test")
    parser.add_argument("--keep-traces", action="store_true",
                        help="archive robot_path and pep csv for each test")
    parser.add_argument("--smoke", action="store_true",
                        help="run only 4 combinations as a sanity check")
    args = parser.parse_args()

    roads = args.roads or ROADS
    widths = args.widths or WIDTHS
    models = args.models or MODELS

    if args.smoke:
        roads, widths = roads[:1], [0.20]
        models = models[:2]
        combos = [(r, w, m) for r in roads for w in widths for m in models]
        combos += [(roads[0], 0.12, models[i]) for i in range(min(2, len(models)))]
    else:
        combos = [(r, w, m) for r in roads for w in widths for m in models]

    csv_path = os.path.join(args.root, args.out)
    done = load_done(csv_path)
    todo = [c for c in combos if (c[0], str(c[1]), c[2]) not in done]

    print("Campaign: %d combinations, %d already done, %d to run."
          % (len(combos), len(combos) - len(todo), len(todo)))
    if not todo:
        return

    is_new = not os.path.exists(csv_path)
    handle = open(csv_path, "a", newline="")
    writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS, extrasaction="ignore")
    if is_new:
        writer.writeheader()
        handle.flush()

    started = time.time()
    counts = {}
    for index, (road, width, model) in enumerate(todo, 1):
        label = "%s w=%.2f %s" % (road, width, model)
        print("[%d/%d] %-34s " % (index, len(todo), label), end="", flush=True)
        verdict = run_one(args.root, args.webots, road, width, model,
                          args.keep_traces, args.timeout)
        if verdict is None:
            print("FAILED")
            continue
        writer.writerow(verdict)
        handle.flush()
        counts[verdict["status"]] = counts.get(verdict["status"], 0) + 1
        print("%-8s f1=%+.4f  (%.0fs)"
              % (verdict["status"], verdict["f1_margin_to_oob"],
                 verdict["wall_seconds"]))

        elapsed = time.time() - started
        remaining = elapsed / index * (len(todo) - index)
        if index % 10 == 0:
            print("      ... %s | remaining ~%.0f min"
                  % (", ".join("%s=%d" % kv for kv in sorted(counts.items())),
                     remaining / 60))

    handle.close()
    print("\nDone in %.1f min. Results: %s"
          % ((time.time() - started) / 60, csv_path))
    for status, count in sorted(counts.items()):
        print("  %-8s %d" % (status, count))


if __name__ == "__main__":
    main()