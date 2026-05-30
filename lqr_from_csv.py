"""
Fit a discrete inverted-pendulum model from CSV data and compute LQR gains.

Required data model:
    x[k+1] = A x[k] + B u[k]

State order used everywhere in this script:
    x = [theta, theta_dot, pos, vel]^T

Input:
    u = pwm

Expected CSV columns can be inferred from names such as:
    theta_rad, theta_dot_radps, pos_m, vel_mps, pwm

or from the fixed-point serial log format:
    theta_mrad, theta_dot_mradps, pos_mm, vel_mmps, pwm

Example:
    python lqr_from_csv.py --csv standup_log.csv

The fitted A/B matrices are discrete-time matrices for the CSV sample interval.
If your controller runs every 10 ms, collect rows every 10 ms when possible.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    from scipy.linalg import solve_discrete_are as scipy_solve_discrete_are
except ImportError:
    scipy_solve_discrete_are = None


STATE_NAMES = ["theta", "theta_dot", "pos", "vel"]


def parse_diag(text: str, expected_len: int, name: str) -> np.ndarray:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} comma-separated values.")
    return np.diag(values)


def parse_reference(text: str) -> np.ndarray:
    values = np.array([float(item.strip()) for item in text.split(",") if item.strip()])
    if values.shape != (4,):
        raise ValueError(
            "--reference must contain 4 comma-separated SI values in order: "
            "theta,theta_dot,pos,vel"
        )
    return values


def read_csv_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError("CSV file has no header row.")
        rows = list(reader)
    if not rows:
        raise ValueError("CSV file has no data rows.")
    return list(reader.fieldnames), rows


def write_csv_rows(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def find_column(columns: list[str], explicit: str | None, candidates: Iterable[str]) -> str:
    if explicit:
        if explicit not in columns:
            raise ValueError(f"Column '{explicit}' was not found in the CSV.")
        return explicit

    normalized = {str(col).strip().lower(): col for col in columns}
    for candidate in candidates:
        key = candidate.strip().lower()
        if key in normalized:
            return str(normalized[key])

    raise ValueError(
        "Could not infer a required CSV column. Available columns: " + ", ".join(columns)
    )


def numeric_column(rows: list[dict[str, str]], column: str) -> np.ndarray:
    values = []
    for row_index, row in enumerate(rows, start=2):
        raw = row.get(column, "")
        try:
            values.append(float(raw))
        except ValueError as exc:
            raise ValueError(
                f"Column '{column}' contains a non-numeric value at CSV row "
                f"{row_index}: {raw!r}"
            ) from exc
    return np.array(values, dtype=float)


def infer_angle_unit(column: str, explicit: str) -> str:
    if explicit != "auto":
        return explicit

    name = column.strip().lower()
    if "mrad" in name:
        return "mrad"
    if "deg" in name:
        return "deg"
    return "rad"


def infer_position_unit(column: str, explicit: str) -> str:
    if explicit != "auto":
        return explicit

    name = column.strip().lower()
    if name.endswith("_mm") or "pos_mm" in name or "position_mm" in name:
        return "mm"
    return "m"


def infer_velocity_unit(column: str, explicit: str) -> str:
    if explicit != "auto":
        return explicit

    name = column.strip().lower()
    if "mmps" in name or "mm_s" in name or "mm/s" in name:
        return "mmps"
    return "mps"


def convert_angle(values: np.ndarray, unit: str) -> np.ndarray:
    if unit == "rad":
        return values
    if unit == "mrad":
        return values / 1000.0
    if unit == "deg":
        return np.deg2rad(values)
    raise ValueError(f"Unsupported angle unit: {unit}")


def convert_position(values: np.ndarray, unit: str) -> np.ndarray:
    if unit == "m":
        return values
    if unit == "mm":
        return values / 1000.0
    raise ValueError(f"Unsupported position unit: {unit}")


def convert_velocity(values: np.ndarray, unit: str) -> np.ndarray:
    if unit == "mps":
        return values
    if unit == "mmps":
        return values / 1000.0
    raise ValueError(f"Unsupported velocity unit: {unit}")


def load_data(
    fieldnames: list[str], rows: list[dict[str, str]], args: argparse.Namespace
) -> tuple[np.ndarray, np.ndarray, dict[str, str]]:
    theta_col = find_column(
        fieldnames,
        args.theta_col,
        ["theta", "theta_rad", "theta_mrad", "angle", "angle_rad", "angle_mrad", "角度"],
    )
    theta_dot_col = find_column(
        fieldnames,
        args.theta_dot_col,
        [
            "theta_dot",
            "theta_dot_radps",
            "theta_dot_mradps",
            "thetadot",
            "omega",
            "angular_velocity",
            "angle_velocity",
            "角速度",
        ],
    )
    pos_col = find_column(
        fieldnames,
        args.pos_col,
        ["pos", "pos_m", "pos_mm", "position", "position_m", "position_mm", "x", "位置"],
    )
    vel_col = find_column(
        fieldnames,
        args.vel_col,
        ["vel", "vel_mps", "vel_mmps", "velocity", "velocity_mps", "velocity_mmps", "v", "速度"],
    )
    pwm_col = find_column(
        fieldnames,
        args.pwm_col,
        ["pwm", "pwm_ticks", "pwmticks", "u", "control", "input", "输出"],
    )

    theta_unit = infer_angle_unit(theta_col, args.theta_unit)
    theta_dot_unit = infer_angle_unit(theta_dot_col, args.theta_dot_unit)
    pos_unit = infer_position_unit(pos_col, args.pos_unit)
    vel_unit = infer_velocity_unit(vel_col, args.vel_unit)

    theta = args.theta_sign * convert_angle(
        numeric_column(rows, theta_col) - args.theta_offset, theta_unit
    )
    theta_dot = args.theta_sign * convert_angle(
        numeric_column(rows, theta_dot_col), theta_dot_unit
    )
    pos = convert_position(numeric_column(rows, pos_col), pos_unit)
    vel = convert_velocity(numeric_column(rows, vel_col), vel_unit)
    pwm = numeric_column(rows, pwm_col)

    states = np.column_stack([theta, theta_dot, pos, vel])
    columns = {
        "theta": theta_col,
        "theta_dot": theta_dot_col,
        "pos": pos_col,
        "vel": vel_col,
        "pwm": pwm_col,
        "theta_unit": theta_unit,
        "theta_dot_unit": theta_dot_unit,
        "pos_unit": pos_unit,
        "vel_unit": vel_unit,
    }
    return states, pwm, columns


def build_valid_sample_mask(
    states: np.ndarray, pwm: np.ndarray, max_angle_deg: float, max_abs_pwm: float
) -> np.ndarray:
    mask = np.isfinite(states).all(axis=1) & np.isfinite(pwm)

    if max_angle_deg > 0.0:
        max_angle_rad = np.deg2rad(max_angle_deg)
        mask &= np.abs(states[:, 0]) <= max_angle_rad

    if max_abs_pwm > 0.0:
        mask &= np.abs(pwm) <= max_abs_pwm

    return mask


def identify_discrete_model(
    states: np.ndarray, pwm: np.ndarray, sample_mask: np.ndarray
) -> tuple[np.ndarray, np.ndarray, int, int, int]:
    """
    Estimate x[k+1] = A x[k] + B pwm[k] using least squares.

    Only adjacent valid sample pairs are used. This avoids accidentally fitting
    a transition across a filtered-out large-angle or saturated sample.
    """

    pair_mask = sample_mask[:-1] & sample_mask[1:]
    used_pairs = int(np.count_nonzero(pair_mask))
    discarded_samples = int(states.shape[0] - np.count_nonzero(sample_mask))
    discarded_pairs = int((states.shape[0] - 1) - used_pairs)

    if used_pairs < 8:
        raise ValueError(
            f"Only {used_pairs} adjacent valid pairs remain. Need more data after filtering."
        )

    x_now = states[:-1, :][pair_mask]
    x_next = states[1:, :][pair_mask]
    u_now = pwm[:-1].reshape(-1, 1)[pair_mask]
    regressor = np.hstack([x_now, u_now])

    coeff, *_ = np.linalg.lstsq(regressor, x_next, rcond=None)
    A = coeff[:4, :].T
    B = coeff[4:, :].T
    return A, B, used_pairs, discarded_samples, discarded_pairs


def lqr_discrete(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    max_iter: int,
    tolerance: float,
) -> tuple[np.ndarray, str, int, float]:
    """
    Solve the discrete-time LQR problem by iterating the Riccati equation.

    K = inv(B.T P B + R) B.T P A
    u = -K x
    """

    if scipy_solve_discrete_are is not None:
        P = scipy_solve_discrete_are(A, B, Q, R)
        K = np.linalg.solve(B.T @ P @ B + R, B.T @ P @ A)
        residual = (
            A.T @ P @ A
            - P
            - A.T @ P @ B @ K
            + Q
        )
        delta = float(np.linalg.norm(residual, ord=np.inf))
        return K, "scipy.solve_discrete_are", 0, delta

    P = Q.copy()
    last_delta = np.inf
    for iteration in range(1, max_iter + 1):
        lhs = B.T @ P @ B + R
        rhs = B.T @ P @ A
        K = np.linalg.solve(lhs, rhs)
        P_next = A.T @ P @ A - A.T @ P @ B @ K + Q
        P_next = 0.5 * (P_next + P_next.T)
        last_delta = float(np.linalg.norm(P_next - P, ord=np.inf))
        P = P_next
        if last_delta < tolerance:
            return K, "iterative_riccati", iteration, last_delta

    raise RuntimeError(
        f"Riccati iteration did not converge after {max_iter} iterations; "
        f"last delta = {last_delta:.6g}. Check A/B/Q/R or increase --max-riccati-iter."
    )


def controllability_rank(A: np.ndarray, B: np.ndarray) -> int:
    blocks = [B]
    current = B
    for _ in range(1, A.shape[0]):
        current = A @ current
        blocks.append(current)
    matrix = np.hstack(blocks)
    return int(np.linalg.matrix_rank(matrix))


def add_output_columns(
    rows: list[dict[str, str]],
    states: np.ndarray,
    pwm: np.ndarray,
    u_lqr: np.ndarray,
) -> tuple[list[str], list[dict[str, str]]]:
    extra_fields = ["theta_rad", "theta_dot_radps", "pos_m", "vel_mps", "pwm_used", "u_lqr"]
    output_rows: list[dict[str, str]] = []

    for row, state, pwm_value, u_value in zip(rows, states, pwm, u_lqr):
        output_row = dict(row)
        output_row["theta_rad"] = f"{state[0]:.10g}"
        output_row["theta_dot_radps"] = f"{state[1]:.10g}"
        output_row["pos_m"] = f"{state[2]:.10g}"
        output_row["vel_mps"] = f"{state[3]:.10g}"
        output_row["pwm_used"] = f"{pwm_value:.10g}"
        output_row["u_lqr"] = f"{u_value:.10g}"
        output_rows.append(output_row)

    return extra_fields, output_rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit x[k+1] = A x[k] + B pwm[k] from CSV data and compute LQR K. "
            "State order is theta,theta_dot,pos,vel."
        )
    )
    parser.add_argument("--csv", required=True, type=Path, help="Input CSV file.")
    parser.add_argument("--out", type=Path, default=None, help="Output CSV file with u_lqr.")

    parser.add_argument("--theta-col", default=None, help="CSV column for theta.")
    parser.add_argument("--theta-dot-col", default=None, help="CSV column for theta_dot.")
    parser.add_argument("--pos-col", default=None, help="CSV column for cart position.")
    parser.add_argument("--vel-col", default=None, help="CSV column for cart velocity.")
    parser.add_argument("--pwm-col", default=None, help="CSV column for actual PWM input.")

    parser.add_argument(
        "--theta-unit",
        choices=["auto", "rad", "mrad", "deg"],
        default="auto",
        help="Unit of theta column. auto infers from column name.",
    )
    parser.add_argument(
        "--theta-dot-unit",
        choices=["auto", "rad", "mrad", "deg"],
        default="auto",
        help="Unit of theta_dot column. rad means rad/s, mrad means mrad/s, deg means deg/s.",
    )
    parser.add_argument(
        "--pos-unit",
        choices=["auto", "m", "mm"],
        default="auto",
        help="Unit of position column. auto infers from column name.",
    )
    parser.add_argument(
        "--vel-unit",
        choices=["auto", "mps", "mmps"],
        default="auto",
        help="Unit of velocity column. auto infers from column name.",
    )
    parser.add_argument(
        "--theta-offset",
        type=float,
        default=0.0,
        help="Raw theta offset at upright equilibrium, before unit conversion.",
    )
    parser.add_argument(
        "--theta-sign",
        type=float,
        choices=[-1.0, 1.0],
        default=1.0,
        help="Set to -1 if the sensor theta direction is opposite to the motor model.",
    )

    parser.add_argument(
        "--q",
        default="300,30,1,5",
        help="Q diagonal in state order: theta,theta_dot,pos,vel.",
    )
    parser.add_argument("--r", default="1", help="R diagonal. For one input, pass one value.")
    parser.add_argument(
        "--reference",
        default="0,0,0,0",
        help="Reference in SI units and order: theta,theta_dot,pos,vel.",
    )
    parser.add_argument(
        "--max-angle-deg",
        type=float,
        default=7.0,
        help="Discard samples whose abs(theta) exceeds this many degrees. Default is 7. Set 0 to disable.",
    )
    parser.add_argument(
        "--max-abs-pwm",
        type=float,
        default=0.0,
        help="Discard samples whose abs(pwm) exceeds this value. Set 0 to disable.",
    )
    parser.add_argument("--u-min", type=float, default=None, help="Optional lower saturation for u_lqr.")
    parser.add_argument("--u-max", type=float, default=None, help="Optional upper saturation for u_lqr.")
    parser.add_argument(
        "--max-riccati-iter",
        type=int,
        default=10000,
        help="Maximum iterations for the discrete Riccati equation.",
    )
    parser.add_argument(
        "--riccati-tol",
        type=float,
        default=1e-10,
        help="Convergence tolerance for the discrete Riccati equation.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    Q = parse_diag(args.q, 4, "--q")
    R = parse_diag(args.r, 1, "--r")
    reference = parse_reference(args.reference)

    fieldnames, rows = read_csv_rows(args.csv)
    states, pwm, columns = load_data(fieldnames, rows, args)
    sample_mask = build_valid_sample_mask(states, pwm, args.max_angle_deg, args.max_abs_pwm)
    A, B, used_pairs, discarded_samples, discarded_pairs = identify_discrete_model(
        states, pwm, sample_mask
    )
    K, solver, iterations, delta = lqr_discrete(
        A, B, Q, R, max_iter=args.max_riccati_iter, tolerance=args.riccati_tol
    )

    errors = states - reference.reshape(1, 4)
    u_lqr = -(errors @ K.T).reshape(-1)

    if args.u_min is not None or args.u_max is not None:
        low = -np.inf if args.u_min is None else args.u_min
        high = np.inf if args.u_max is None else args.u_max
        if low > high:
            raise ValueError("--u-min must be less than or equal to --u-max.")
        u_lqr = np.clip(u_lqr, low, high)

    out_path = args.out
    if out_path is None:
        out_path = args.csv.with_name(args.csv.stem + "_lqr.csv")

    extra_fields, output_rows = add_output_columns(rows, states, pwm, u_lqr)
    output_fields = list(fieldnames)
    for field in extra_fields:
        if field not in output_fields:
            output_fields.append(field)
    write_csv_rows(out_path, output_fields, output_rows)

    rank = controllability_rank(A, B)
    np.set_printoptions(precision=8, suppress=True)
    print("Identified model: x[k+1] = A x[k] + B pwm[k]")
    print("State order: [theta, theta_dot, pos, vel]")
    print("Units used for fitting: theta rad, theta_dot rad/s, pos m, vel m/s, pwm ticks")
    print(f"CSV columns used: {columns}")
    print(f"Valid adjacent pairs used: {used_pairs}")
    print(f"Discarded samples: {discarded_samples}, discarded adjacent pairs: {discarded_pairs}")
    print(f"Controllability rank: {rank}/4")
    if iterations > 0:
        print(f"LQR solver: {solver}, iterations: {iterations}, final delta: {delta:.6g}")
    else:
        print(f"LQR solver: {solver}, Riccati residual: {delta:.6g}")
    print("A =")
    print(A)
    print("B =")
    print(B)
    print("K =")
    print(K)
    print()
    print("C defines:")
    print(f"#define LQR_K_THETA     ({K[0, 0]:.10g}f)")
    print(f"#define LQR_K_THETA_DOT ({K[0, 1]:.10g}f)")
    print(f"#define LQR_K_POS       ({K[0, 2]:.10g}f)")
    print(f"#define LQR_K_VEL       ({K[0, 3]:.10g}f)")
    print()
    print("C control law:")
    print("pwm = -(LQR_K_THETA * theta_rad +")
    print("        LQR_K_THETA_DOT * theta_dot_radps +")
    print("        LQR_K_POS * pos_m +")
    print("        LQR_K_VEL * vel_mps);")
    print(f"Output written to: {out_path}")


if __name__ == "__main__":
    main()
