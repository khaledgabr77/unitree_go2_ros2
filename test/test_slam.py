"""SLAM accuracy check in sim, one Gazebo run per available back-end."""

import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path
from subprocess import Popen
from typing import Literal

import pytest
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

pytestmark = pytest.mark.skipif(
    shutil.which("evo_ape") is None,
    reason="evo is not installed. You can install it with uv: `uv tool install evo`.",
)

BAGS = Path(__file__).resolve().parents[1] / "bag"
CMD_BAGS = ["cmd_vel"]
EVO = {"ape": "evo_ape", "rpe": "evo_rpe"}
GT = "/odom/ground_truth"
SLAM = "/slam_pose"

BACKEND_PKG = {"slam_toolbox": "slam_toolbox", "gmapping": "gmapper", "rtabmap": "rtabmap_slam"}

APE_MAX_THRESHOLD = {"slam_toolbox": 0.5, "rtabmap": 0.5, "gmapping": 0.5}


def sim_cmd(backend: Literal["slam_toolbox", "gmapping", "rtabmap"]):
    return [
        "ros2",
        "launch",
        "unitree_indoor_slam",
        "unitree_sim_slam.launch.py",
        f"slam:={backend}",
        f"world:={os.path.join(get_package_share_directory('unitree_go2_description'), 'worlds', 'default.sdf')}",
        # The launch file defaults to the cave entrance (x=-30), out of scan range of every default.sdf obstacle.
        "world_init_x:=0.0",
        "world_init_z:=0.375",
        "rviz:=false",
        "gui:=false",
        "disable_camera:=true",
        "disable_d455:=true",
        "disable_lidar_l1:=true",
        "slam_start_delay:=20.0",
    ]


def rec_cmd(out: str):
    return ["ros2", "bag", "record", "-o", out, "--topics", "/odom", GT, SLAM, "--use-sim-time"]


def bag_play_cmd(bag: str):
    return ["ros2", "bag", "play", str(BAGS / bag), "--rate", "1"]


relay_cmd = ["ros2", "run", "unitree_application", "tf_to_odom_node", "--ros-args", "-p", "use_sim_time:=true"]


def backend_params() -> list[pytest.param]:
    """One param per (cmd bag, back-end), skipped when the back-end is not built."""
    installed = get_packages_with_prefixes()
    return [
        pytest.param(
            (cmd_bag, backend),
            marks=pytest.mark.skipif(pkg not in installed, reason=f"{pkg} is not built"),
            id=f"{cmd_bag}-{backend}",
        )
        for cmd_bag in CMD_BAGS
        for backend, pkg in BACKEND_PKG.items()
    ]


def wait_for_log(process: Popen[str], pattern: str, timeout: float = 30) -> bool:
    """Wait for the pattern in the process output."""
    assert process.stdout is not None
    start_time = time.time()
    for line in iter(process.stdout.readline, ""):
        if line and re.search(pattern, line):
            return True
        if time.time() - start_time > timeout:
            raise TimeoutError(f"Timed out waiting for pattern: {pattern}")
    return False


def wait_for_topic(topic: str, timeout: float = 180) -> None:
    """Block until the topic carries a message."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            echo = subprocess.run(
                ["ros2", "topic", "echo", topic, "--once"], capture_output=True, text=True, check=False, timeout=15
            )
        except subprocess.TimeoutExpired:
            continue
        if echo.returncode == 0:
            return
    raise TimeoutError(f"Timed out waiting for a message on {topic}")


def ensure_slam_toolbox_active(timeout: float = 60) -> None:
    """Activate slam_toolbox when the launch file's own transition event was lost."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        state = subprocess.run(
            ["ros2", "lifecycle", "get", "/slam_toolbox"], capture_output=True, text=True, check=False, timeout=15
        )
        if state.stdout.startswith("active"):
            return
        if state.stdout.startswith("inactive"):
            subprocess.run(
                ["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                capture_output=True,
                text=True,
                check=False,
                timeout=15,
            )
        time.sleep(3)


def get_evo_max_error(bag_path: str, ref_topic: str, est_topic: str, mode: Literal["ape", "rpe"] = "ape") -> float:
    """Launch evo on a bag2, return max error.

    --align_origin is not optional: /odom/ground_truth comes out of the Gazebo
    odometry plugin in world coordinates (it starts at world_init_x), while the
    SLAM map and /odom are both anchored at the spawn pose.
    """
    result = subprocess.run(
        [EVO[mode], "bag2", bag_path, ref_topic, est_topic, "-r", "trans_part", "--align_origin"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Evo failed: {result.stderr}")
    match = re.search(r"max\s+([\d.]+)", result.stdout)
    if match:
        return float(match.group(1))
    raise RuntimeError(f"Regex failed on evo output: {result.stdout}")


@pytest.fixture(scope="module", params=backend_params())
def shared_simulation(request: pytest.FixtureRequest) -> tuple[str, str]:
    """Runs the Gazebo simulation with one SLAM back-end and records the bag file."""
    cmd_bag, backend = request.param
    odom_bag = os.path.join("bag", f"eval_{cmd_bag}_{backend}")
    log_path = os.path.join("log", f"eval_{cmd_bag}_{backend}_sim.log")

    if os.path.exists(odom_bag):
        shutil.rmtree(odom_bag)

    os.makedirs(os.path.dirname(log_path), exist_ok=True)

    sim: Popen[str] | None = None
    relay: Popen[str] | None = None
    record: Popen[str] | None = None
    play: Popen[str] | None = None

    with open(log_path, "w") as sim_log:
        try:
            sim = Popen(sim_cmd(backend), start_new_session=True, stdout=sim_log, stderr=subprocess.STDOUT, text=True)
            print(f"\nWaiting for the simulation and {backend} to come up...")

            relay = Popen(
                relay_cmd, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, text=True
            )
            if backend == "slam_toolbox":
                ensure_slam_toolbox_active()
            wait_for_topic(SLAM)

            record = Popen(
                rec_cmd(odom_bag), start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            print("Waiting for rosbag2 recorder to stop discovery...")
            wait_for_log(record, r"Topics discovery stopped")
            time.sleep(2)

            print(f"Executing rosbag2 player: {cmd_bag}")
            play = subprocess.Popen(bag_play_cmd(cmd_bag), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            play_out, _ = play.communicate()
            if play.returncode != 0:
                raise RuntimeError(f"ros2 bag play failed:\n{play_out}")
            time.sleep(1)

        finally:
            print("Shutting down Gazebo and Recording...")
            for proc in [record, play, relay, sim]:
                if proc and proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            print("Cleanup complete")

    return odom_bag, backend


def test_slam_error(shared_simulation: tuple[str, str]) -> None:
    """Checks the APE of the SLAM pose estimate against Gazebo ground truth."""
    odom_bag, backend = shared_simulation
    threshold = APE_MAX_THRESHOLD[backend]
    error = get_evo_max_error(odom_bag, GT, SLAM)
    print(f"{backend} APE is {error}")
    assert error < threshold, (
        f"{backend}: transformation map -> base_link absolute pose error {error} exceeded threshold {threshold}"
    )
