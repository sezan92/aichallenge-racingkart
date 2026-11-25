#!/usr/bin/env python3
"""
Optuna-based parameter sweep for Autoware MPC / Pure Pursuit controllers.

Usage:
    python3 optimize.py --controller mpc --n-trials 50 --study-name mpc_tuning
    python3 optimize.py --controller pure_pursuit --n-trials 30
"""

import argparse
import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Dict, Optional

import optuna
import yaml
import re

# ── Paths ──────────────────────────────────────────────────────
AICHALLENGE_DIR = Path("/aichallenge")
WORKSPACE_DIR = AICHALLENGE_DIR / "workspace"
MPC_CONFIG_PATH = (
    WORKSPACE_DIR / "src" / "aichallenge_submit"
    / "multi_purpose_mpc_ros" / "config" / "config.yaml"
)
PURE_PURSUIT_LAUNCH = (
    WORKSPACE_DIR / "src" / "aichallenge_submit"
    / "aichallenge_submit_launch" / "launch" / "control"
    / "pure_pursuit.launch.xml"
)
RESULT_DETAILS_PATH = AICHALLENGE_DIR / "d1-result-details.json"
RESULT_SUMMARY_PATH = AICHALLENGE_DIR / "result-summary.json"
OUTPUT_BASE = Path("/output/optuna_sweeps")


# ── MPC Parameter Space ───────────────────────────────────────
def suggest_mpc_params(trial: optuna.Trial) -> Dict:
    return {
        "v_max": trial.suggest_float("v_max", 15.0, 40.0),
        "ay_max": trial.suggest_float("ay_max", 3.0, 15.0),
        "Q0": trial.suggest_float("Q0", 5e5, 1e7, log=True),
        "Q1": trial.suggest_float("Q1", 1e7, 2e8, log=True),
        "Q2": trial.suggest_float("Q2", 1e4, 5e6, log=True),
        "QN0": trial.suggest_float("QN0", 1e5, 5e6, log=True),
        "R0": trial.suggest_float("R0", 1e4, 5e5, log=True),
        "N": trial.suggest_int("N", 10, 40),
        "accel_low_pass_gain": trial.suggest_float("accel_low_pass_gain", 0.1, 1.0),
        "steer_low_pass_gain": trial.suggest_float("steer_low_pass_gain", 0.1, 1.0),
        "wp_id_offset": trial.suggest_int("wp_id_offset", 0, 6),
    }


# ── Pure Pursuit Parameter Space ──────────────────────────────
def suggest_pure_pursuit_params(trial: optuna.Trial) -> Dict:
    return {
        "lookahead_gain": trial.suggest_float("lookahead_gain", 0.2, 2.0),
        "lookahead_min_distance": trial.suggest_float("lookahead_min_distance", 1.0, 8.0),
        "speed_proportional_gain": trial.suggest_float("speed_proportional_gain", 0.3, 3.0),
        "steering_tire_angle_gain": trial.suggest_float("steering_tire_angle_gain", 1.0, 2.0),
    }


# ── Config Writers ────────────────────────────────────────────
def write_mpc_config(params: Dict, backup_path: Path) -> None:
    """Write MPC config.yaml with trial parameters."""
    with open(MPC_CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f)

    # Backup original
    shutil.copy2(MPC_CONFIG_PATH, backup_path)

    config["mpc"]["v_max"] = params["v_max"]
    config["mpc"]["ay_max"] = params["ay_max"]
    config["mpc"]["Q"] = [params["Q0"], params["Q1"], params["Q2"]]
    config["mpc"]["QN"] = [params["QN0"], params["QN0"], params["QN0"]]
    # QN[1] and QN[2] kept fixed at 1000 and 10000 (as in existing configs)
    config["mpc"]["QN"][1] = 1000.0
    config["mpc"]["QN"][2] = 10000.0
    config["mpc"]["R"] = [params["R0"], 0.0]
    config["mpc"]["N"] = params["N"]
    config["mpc"]["accel_low_pass_gain"] = params["accel_low_pass_gain"]
    config["mpc"]["steer_low_pass_gain"] = params["steer_low_pass_gain"]
    config["mpc"]["wp_id_offset"] = params["wp_id_offset"]

    with open(MPC_CONFIG_PATH, "w") as f:
        yaml.dump(config, f, default_flow_style=False)


def write_pure_pursuit_launch(params: Dict, backup_path: Path) -> None:
    """Write pure_pursuit.launch.xml with trial parameters."""
    shutil.copy2(PURE_PURSUIT_LAUNCH, backup_path)

    content = PURE_PURSUIT_LAUNCH.read_text()
    # Replace param values in XML using simple string replacement
    content = _replace_xml_param(content, "lookahead_gain", params["lookahead_gain"])
    content = _replace_xml_param(content, "lookahead_min_distance", params["lookahead_min_distance"])
    content = _replace_xml_param(content, "speed_proportional_gain", params["speed_proportional_gain"])
    # steering_tire_angle_gain for simulation
    content = content.replace(
        'value="1.50" if="$(var simulation)"',
        f'value="{params["steering_tire_angle_gain"]}" if="$(var simulation)"'
    )
    PURE_PURSUIT_LAUNCH.write_text(content)


# ── Evaluation Runner ─────────────────────────────────────────
def run_evaluation(
    controller: str,
    timeout_sec: int = 900,
    num_laps: int = 3,
) -> Optional[Dict]:
    """
    Launch AWSIM + Autoware for one trial and collect results.

    This runs the evaluation pipeline:
    1. Start AWSIM simulator (eval mode)
    2. Start Autoware with the selected controller
    3. Wait for completion
    4. Parse result JSON files
    """
    trial_output_dir = OUTPUT_BASE / f"trial_{int(time.time())}"
    trial_output_dir.mkdir(parents=True, exist_ok=True)

    # Clear previous results
    for result_file in [RESULT_DETAILS_PATH, RESULT_SUMMARY_PATH]:
        if result_file.exists():
            result_file.unlink()

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "1"
    env["SIM_MODE"] = "eval"

    # Build the evaluation command
    # This uses the existing run_evaluation.bash which bundles
    # simulator + autoware + orchestrator
    cmd = [
        "bash", str(AICHALLENGE_DIR / "run_evaluation.bash"),
    ]

    try:
        proc = subprocess.run(
            cmd,
            env=env,
            timeout=timeout_sec,
            capture_output=True,
            text=True,
        )
    except subprocess.TimeoutExpired:
        print(f"[WARN] Trial timed out after {timeout_sec}s")
        return None

    # Parse results
    return parse_results(trial_output_dir)


def parse_results(output_dir: Path) -> Optional[Dict]:
    """Parse d1-result-details.json and compute metrics."""
    if not RESULT_DETAILS_PATH.exists():
        print("[ERROR] No result-details.json found")
        return None

    with open(RESULT_DETAILS_PATH) as f:
        details = json.load(f)

    # Save a copy
    shutil.copy2(RESULT_DETAILS_PATH, output_dir / "result-details.json")

    result = {
        "finished": details["finished"],
        "lap_count": details["lap_count"],
        "min_lap_time": details["min_lap_time"],
        "avg_lap_time": details["avg_lap_time"],
        "total_lap_time": details["total_lap_time"],
        "penalty_count": details["penalty_count"],
        "penalty_total_seconds": details["penalty_total_seconds"],
    }

    return result

def objective(trial: optuna.Trial, controller: str, metric: str) -> float:
    """Optuna objective: minimize the chosen metric."""
    # 1. Suggest parameters
    if controller == "mpc":
        params = suggest_mpc_params(trial)
    else:
        params = suggest_pure_pursuit_params(trial)

    # 2. Backup & write config
    backup_path = Path(f"/tmp/{controller}_backup_trial_{trial.number}")
    try:
        if controller == "mpc":
            write_mpc_config(params, backup_path)
        else:
            write_pure_pursuit_launch(params, backup_path)

        # 3. Rebuild if needed (for pure_pursuit XML changes)
        if controller == "pure_pursuit":
            _rebuild_workspace()

        # 4. Run evaluation
        result = run_evaluation(controller)

    finally:
        # 5. Restore original config
        if backup_path.exists():
            target = MPC_CONFIG_PATH if controller == "mpc" else PURE_PURSUIT_LAUNCH
            shutil.copy2(backup_path, target)

    if result is None:
        return float("inf")  # Failed trial

    # 6. Compute objective value
    if metric == "lap_time":
        # Primary: minimize best lap time
        if result["lap_count"] == 0:
            return float("inf")  # No lap completed
        return result["min_lap_time"]

    elif metric == "composite":
        # Multi-objective: lap_time + penalty + consistency
        PENALTY_SCORE_WEIGHT=5.0
        CONSISTENCY_WEIGHT=2.0
        if result["lap_count"] == 0:
            return float("inf")
        lap_score = result["min_lap_time"]
        penalty_score = result["penalty_total_seconds"] * PENALTY_SCORE_WEIGHT  # heavy penalty
        consistency = (
            result["avg_lap_time"] - result["min_lap_time"]
        ) * CONSISTENCY_WEIGHT  # penalize inconsistency
        return lap_score + penalty_score + consistency

    elif metric == "avg_lap_time":
        if result["lap_count"] == 0:
            return float("inf")
        return result["avg_lap_time"]

    return float("inf")


# ── Helpers ───────────────────────────────────────────────────
def _replace_xml_param(xml: str, name: str, value: float) -> str:
    pattern = rf'(<param\s+name="{name}"\s+value=")[^"]*(")'
    return re.sub(pattern, rf'\g<1>{value}\2', xml)


def _rebuild_workspace() -> None:
    """Rebuild the overlay workspace (needed after XML changes)."""
    subprocess.run(
        ["bash", str(AICHALLENGE_DIR / "build_autoware.bash")],
        check=True,
        capture_output=True,
    )


# ── Main ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Optuna parameter sweep for Autoware controllers"
    )
    parser.add_argument(
        "--controller",
        choices=["mpc", "pure_pursuit"],
        default="mpc",
        help="Controller to optimize",
    )
    parser.add_argument(
        "--n-trials", type=int, default=50,
        help="Number of Optuna trials",
    )
    parser.add_argument(
        "--metric",
        choices=["lap_time", "avg_lap_time", "composite"],
        default="lap_time",
        help="Objective metric to minimize",
    )
    parser.add_argument(
        "--study-name",
        default=None,
        help="Optuna study name (auto-generated if omitted)",
    )
    parser.add_argument(
        "--storage",
        default=None,
        help="Optuna storage URL (e.g., sqlite:///optuna.db)",
    )
    parser.add_argument(
        "--timeout", type=int, default=900,
        help="Per-trial timeout in seconds",
    )
    args = parser.parse_args()

    study_name = args.study_name or f"{args.controller}_{args.metric}"
    storage = args.storage or f"sqlite:///{OUTPUT_BASE}/{study_name}.db"
    OUTPUT_BASE.mkdir(parents=True, exist_ok=True)

    # Create or load study
    study = optuna.create_study(
        study_name=study_name,
        storage=storage,
        direction="minimize",
        load_if_exists=True,
        sampler=optuna.samplers.TPESampler(
            n_startup_trials=10,
            multivariate=True,
        ),
    )

    # Callback to log results
    def log_callback(study, trial):
        print(
            f"\n{'='*60}\n"
            f"Trial {trial.number} finished: value={trial.value:.3f}\n"
            f"Best so far: {study.best_value:.3f} "
            f"(trial {study.best_trial.number})\n"
            f"Params: {trial.params}\n"
            f"{'='*60}\n"
        )

    # Run optimization
    study.optimize(
        lambda trial: objective(trial, args.controller, args.metric),
        n_trials=args.n_trials,
        callbacks=[log_callback],
        show_progress_bar=True,
    )

    # Print summary
    print("\n" + "=" * 60)
    print("OPTIMIZATION COMPLETE")
    print("=" * 60)
    print(f"Best trial: #{study.best_trial.number}")
    print(f"Best value: {study.best_value:.3f}")
    print("Best parameters:")
    for k, v in study.best_trial.params.items():
        print(f"  {k}: {v}")

    # Save best params to YAML
    best_params_path = OUTPUT_BASE / f"best_{study_name}.yaml"
    with open(best_params_path, "w") as f:
        yaml.dump(study.best_trial.params, f)
    print(f"\nBest parameters saved to: {best_params_path}")

    # Optional: Generate visualization plots
    try:
        fig = optuna.visualization.plot_optimization_history(study)
        fig.write_html(str(OUTPUT_BASE / f"{study_name}_history.html"))
        fig = optuna.visualization.plot_param_importances(study)
        fig.write_html(str(OUTPUT_BASE / f"{study_name}_importances.html"))
        fig = optuna.visualization.plot_parallel_coordinate(study)
        fig.write_html(str(OUTPUT_BASE / f"{study_name}_parallel.html"))
        print(f"Visualization plots saved to: {OUTPUT_BASE}/")
    except Exception as e:
        print(f"[WARN] Could not generate plots: {e}")


if __name__ == "__main__":
    main()
