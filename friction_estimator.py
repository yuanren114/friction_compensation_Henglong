import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from scipy import signal


DEFAULT_TIME_COL = "t[s]"
DEFAULT_TORQUE_COL = "API_2ms.HwTrq_Nm_s16p10[]"
DEFAULT_ANGLE_COL = "API_2ms.HwAng_Deg_s32p16[]"
DEFAULT_OMEGA_COL = "API_2ms.HwAngVel_Degs_s32p16[]"
DEFAULT_MOTOR_INPUT_COL = "AimiCurrent[]"
FIXED_FILTER_TS = 0.1


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def derive_aux_output_paths(output_csv_path):
    abs_output = os.path.abspath(output_csv_path)
    out_dir = os.path.dirname(abs_output)
    stem = os.path.splitext(os.path.basename(abs_output))[0]
    summary_path = os.path.join(out_dir, f"{stem}_summary.json")
    loss_history_path = os.path.join(out_dir, f"{stem}_loss_history.csv")
    return summary_path, loss_history_path


def sanitize_run_name(name):
    safe = []
    for ch in str(name):
        if ch.isalnum() or ch in ("-", "_", "."):
            safe.append(ch)
        else:
            safe.append("_")
    out = "".join(safe).strip("._")
    return out or "run"


def build_default_run_name(args):
    data_tag = "synthetic" if not args.data else os.path.splitext(os.path.basename(args.data))[0]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return sanitize_run_name(f"{timestamp}_{data_tag}_{args.mode}_{args.omega_source}")


def resolve_output_layout(args):
    run_name = sanitize_run_name(args.run_name) if args.run_name else build_default_run_name(args)
    run_dir = os.path.join("outputs", "runs", run_name)
    output = args.output or os.path.join(run_dir, "results_friction.csv")
    plot_dir = args.plot_dir or os.path.join(run_dir, "plots")
    return run_name, run_dir, output, plot_dir


@dataclass
class SmoothingConfig:
    median_kernel: int = 11
    savgol_window: int = 31
    savgol_polyorder: int = 3
    lowpass_cutoff_hz: float = 4.0
    lowpass_order: int = 2
    mad_clip_k: float = 5.0


def load_data(path):
    ext = os.path.splitext(path)[1].lower()
    try:
        if ext == ".xls":
            return pd.read_excel(path, engine="xlrd")
        return pd.read_excel(path)
    except ImportError as exc:
        message = str(exc)
        if ext == ".xls" and "xlrd" in message.lower():
            raise RuntimeError(
                "Failed to read .xls file. pandas requires the optional dependency "
                "'xlrd' for Excel 97-2003 files. Install it with "
                "`pip install xlrd` or `conda install xlrd`, then rerun."
            ) from exc
        raise
    except ValueError as exc:
        message = str(exc)
        if ext == ".xls" and "xlrd" in message.lower():
            raise RuntimeError(
                "Failed to read .xls file. pandas requires the optional dependency "
                "'xlrd' for Excel 97-2003 files. Install it with "
                "`pip install xlrd` or `conda install xlrd`, then rerun."
            ) from exc
        raise


def _normalize_col_name(value):
    if value is None:
        return ""
    text = str(value).strip()
    return " ".join(text.split())


def _read_excel_with_optional_engine(path, header):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".xls":
        return pd.read_excel(path, engine="xlrd", header=header)
    return pd.read_excel(path, header=header)


def find_header_row(path, target_columns, max_scan_rows=20):
    target_norm = {_normalize_col_name(col) for col in target_columns}
    for header_row in range(max_scan_rows + 1):
        try:
            df_try = _read_excel_with_optional_engine(path, header=header_row)
        except Exception:
            continue
        cols_norm = {_normalize_col_name(col) for col in df_try.columns}
        if target_norm.issubset(cols_norm):
            return header_row, df_try
    return None, None


def generate_synthetic_data(duration_s=20.0, sample_time=0.002, seed=7):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, duration_s, sample_time)
    theta_deg = 90.0 * np.sin(2 * np.pi * 0.15 * t) + 15.0 * np.sin(2 * np.pi * 0.9 * t)
    theta_rad = np.deg2rad(theta_deg)
    omega_true = np.gradient(theta_rad, sample_time)
    alpha_true = np.gradient(omega_true, sample_time)
    motor_input = 1.6 * np.sin(2 * np.pi * 0.15 * t + 0.3) + 0.25 * np.sin(2 * np.pi * 1.7 * t)
    friction_abs = 0.55 + 0.08 * np.tanh(np.abs(omega_true) / 0.6) + 0.02 * np.abs(omega_true)
    friction_true = np.tanh(omega_true / 0.06) * friction_abs
    J_true = 0.0125
    B_true = 0.08
    torque = 0.5 * motor_input + B_true * omega_true + J_true * alpha_true + friction_true
    torque += 0.05 * rng.standard_normal(size=t.shape)
    omega_measured_deg_s = np.rad2deg(omega_true + 0.03 * rng.standard_normal(size=t.shape))
    theta_measured_deg = theta_deg + 0.2 * rng.standard_normal(size=t.shape)
    spike_idx = rng.choice(len(t), size=max(3, len(t) // 350), replace=False)
    torque[spike_idx] += rng.normal(0.0, 0.35, size=spike_idx.shape[0])

    df = pd.DataFrame(
        {
            DEFAULT_TIME_COL: t,
            DEFAULT_TORQUE_COL: torque,
            DEFAULT_OMEGA_COL: omega_measured_deg_s,
            DEFAULT_ANGLE_COL: theta_measured_deg,
            DEFAULT_MOTOR_INPUT_COL: motor_input,
        }
    )
    print("No data file provided. Generated synthetic demo dataset.")
    return df


def detect_columns(df, args):
    columns = list(df.columns)
    required = {
        "time_col": args.time_col or DEFAULT_TIME_COL,
        "torque_col": args.torque_col or DEFAULT_TORQUE_COL,
        "angle_col": args.angle_col or DEFAULT_ANGLE_COL,
        "omega_col": args.omega_col or DEFAULT_OMEGA_COL,
        "motor_input_col": args.motor_input_col or DEFAULT_MOTOR_INPUT_COL,
    }
    missing = [value for value in required.values() if value not in columns]
    if missing:
        raise KeyError(
            "Missing required columns: "
            + ", ".join(missing)
            + "\nAvailable columns: "
            + ", ".join(map(str, columns))
        )
    print("Detected columns:")
    for key, value in required.items():
        print(f"  {key}: {value}")
    return required


def clean_numeric_data(df, columns):
    df_clean = df.copy()
    ordered_cols = [
        columns["time_col"],
        columns["torque_col"],
        columns["angle_col"],
        columns["omega_col"],
        columns["motor_input_col"],
    ]
    for col in ordered_cols:
        df_clean[col] = pd.to_numeric(df_clean[col], errors="coerce")
    before = len(df_clean)
    df_clean = df_clean.dropna(subset=ordered_cols).reset_index(drop=True)
    dropped = before - len(df_clean)
    if dropped > 0:
        print(f"Dropped {dropped} row(s) with NaN/non-numeric values in required columns.")
    if len(df_clean) < 2:
        raise ValueError("Not enough valid numeric samples after cleaning.")
    return df_clean


def filtered_derivative(x, Ts, T_filter=0.1, K=1.0):
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return np.array([], dtype=float)
    filtered_state = np.zeros_like(x)
    derivative = np.zeros_like(x)
    filtered_state[0] = x[0]
    derivative[0] = 0.0
    a = Ts / T_filter
    b = 1.0 - a
    for n in range(1, len(x)):
        derivative[n] = (K / T_filter) * (x[n] - filtered_state[n - 1])
        filtered_state[n] = a * x[n] + b * filtered_state[n - 1]
    return derivative


def compute_nominal_inertia():
    # Geometry grouping is only approximately reconstructed from the Simulink structure.
    # The purpose here is a documented geometry-based estimate and a sanity check against
    # the prior structural estimate J_nominal ~= 0.01275 kg*m^2.
    rho_steel = 0.00785
    rho_aluminum = 0.0027

    def solid_cylinder(rho, radius_mm, length_mm):
        mass_g = rho * math.pi * radius_mm**2 * length_mm
        inertia_g_mm2 = 0.5 * mass_g * radius_mm**2
        return inertia_g_mm2 * 1e-9

    def hollow_cylinder(rho, rin_mm, rout_mm, length_mm):
        mass_g = rho * math.pi * (rout_mm**2 - rin_mm**2) * length_mm
        inertia_g_mm2 = 0.5 * mass_g * (rout_mm**2 + rin_mm**2)
        return inertia_g_mm2 * 1e-9

    parts = {
        "steel_shaft_main": solid_cylinder(rho_steel, 7.85, 281.0),
        "steel_shaft_secondary": solid_cylinder(rho_steel, 10.5, 166.0),
        "al_hollow_section": hollow_cylinder(rho_aluminum, 10.0, 13.0, 166.0),
        "steel_hollow_section": hollow_cylinder(rho_steel, 9.7, 11.2, 166.0),
        # Ambiguous large annular term interpreted as an annular disk of thickness T.
        "steel_annular_disk": hollow_cylinder(rho_steel, 154.5, 190.5, 4.0),
    }
    J_geom = sum(parts.values())
    print("Geometry-based nominal inertia estimate (approximate):")
    for name, value in parts.items():
        print(f"  {name}: {value:.8f} kg*m^2")
    print(f"  J_geom_total: {J_geom:.8f} kg*m^2")
    return J_geom


def prepare_signals(df, columns, args):
    time = df[columns["time_col"]].to_numpy(dtype=float)
    if len(time) < 2:
        raise ValueError("Need at least two samples.")
    dt_est = float(np.median(np.diff(time)))
    Ts = FIXED_FILTER_TS
    print(f"Estimated sample time from data: {dt_est:.6f} s")
    print(f"Using fixed filtered-derivative Ts: {Ts:.6f} s")
    print(
        f"Using derivative filter time T from CLI/default: "
        f"{args.derivative_filter_time:.6f} s"
    )

    angle = df[columns["angle_col"]].to_numpy(dtype=float)
    omega_measured = df[columns["omega_col"]].to_numpy(dtype=float)
    if args.angle_unit == "deg":
        theta_rad = np.deg2rad(angle)
        omega_measured_rad_s = np.deg2rad(omega_measured)
    else:
        theta_rad = angle
        omega_measured_rad_s = omega_measured

    omega_from_angle_rad_s = filtered_derivative(
        theta_rad, Ts=Ts, T_filter=args.derivative_filter_time, K=1.0
    )
    if args.omega_source == "measured":
        omega_used = omega_measured_rad_s
        alpha_used = filtered_derivative(
            omega_measured_rad_s,
            Ts=Ts,
            T_filter=args.derivative_filter_time,
            K=1.0,
        )
    else:
        omega_used = omega_from_angle_rad_s
        alpha_used = filtered_derivative(
            omega_from_angle_rad_s,
            Ts=Ts,
            T_filter=args.derivative_filter_time,
            K=1.0,
        )

    tas_torque = df[columns["torque_col"]].to_numpy(dtype=float)
    motor_input = df[columns["motor_input_col"]].to_numpy(dtype=float)
    motor_contribution = 0.5 * motor_input

    print(
        "Using dynamics residual: friction = TAS_Torque - 0.5*AimiCurrent - B*omega - J*alpha"
    )
    print(
        "AimiCurrent[] is treated as the Simulink-equivalent motor-side input before the 0.5 gain."
    )
    if not args.use_B:
        print("B term disabled: using friction = TAS_Torque - 0.5*AimiCurrent - J*alpha")
    if abs(dt_est - Ts) < 5e-3:
        print("Measured sample time is close to the fixed derivative Ts.")
    else:
        print(
            "Note: measured time spacing and filtered-derivative Ts differ. "
            "This run uses the fixed Ts requested for the derivative block."
        )

    return {
        "time": time,
        "Ts": Ts,
        "dt_est": dt_est,
        "theta_rad": theta_rad,
        "omega_measured_rad_s": omega_measured_rad_s,
        "omega_from_angle_rad_s": omega_from_angle_rad_s,
        "omega_used_rad_s": omega_used,
        "alpha_used_rad_s2": alpha_used,
        "tas_torque": tas_torque,
        "motor_input_AimiCurrent": motor_input,
        "motor_contribution_0p5_AimiCurrent": motor_contribution,
    }


def mad_clip(signal_in, k=5.0):
    signal_in = np.asarray(signal_in, dtype=float)
    med = np.median(signal_in)
    mad = np.median(np.abs(signal_in - med))
    scale = 1.4826 * mad + 1e-9
    lo = med - k * scale
    hi = med + k * scale
    return np.clip(signal_in, lo, hi)


def robust_smooth(signal_in, config, Ts):
    x = mad_clip(signal_in, config.mad_clip_k)
    kernel = max(3, int(config.median_kernel) | 1)
    x_med = signal.medfilt(x, kernel_size=kernel)
    window = max(5, int(config.savgol_window) | 1)
    if window >= len(x_med):
        window = max(5, (len(x_med) - 1) | 1)
    if window <= config.savgol_polyorder:
        window = config.savgol_polyorder + 3
        window = window | 1
    if window >= len(x_med):
        x_sg = x_med.copy()
    else:
        x_sg = signal.savgol_filter(x_med, window, config.savgol_polyorder, mode="interp")
    nyq = 0.5 / Ts
    normalized_cutoff = min(0.99, max(1e-4, config.lowpass_cutoff_hz / nyq))
    b, a = signal.butter(config.lowpass_order, normalized_cutoff, btype="low")
    x_lp = signal.filtfilt(b, a, x_sg)
    return x_lp


def direct_friction_estimate(prepared, J, B, smoothing_config):
    friction_raw = (
        prepared["tas_torque"]
        - prepared["motor_contribution_0p5_AimiCurrent"]
        - B * prepared["omega_used_rad_s"]
        - J * prepared["alpha_used_rad_s2"]
    )
    friction_direct_smoothed = robust_smooth(friction_raw, smoothing_config, prepared["Ts"])
    return friction_raw, friction_direct_smoothed


def build_speed_bins(abs_omega, num_bins=25):
    max_speed = float(np.max(abs_omega)) if len(abs_omega) else 1.0
    max_speed = max(max_speed, 1e-3)
    edges = np.linspace(0.0, max_speed, num_bins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    return edges, centers


class FrictionMagnitudeModel(nn.Module):
    def __init__(self, max_abs_omega):
        super().__init__()
        self.max_abs_omega = float(max(max_abs_omega, 1e-3))
        # Analytic magnitude law:
        # F_abs(|omega|) = F0 + A * (1 - exp(-|omega| / v0)) + C1 * |omega|
        self.raw_F0 = nn.Parameter(torch.tensor(math.log(math.exp(1.00) - 1.0)))
        self.raw_A = nn.Parameter(torch.tensor(math.log(math.exp(0.05) - 1.0)))
        self.raw_v0 = nn.Parameter(torch.tensor(math.log(math.exp(0.50) - 1.0)))
        self.raw_C1 = nn.Parameter(torch.tensor(math.log(math.exp(0.01) - 1.0)))

    def forward(self, abs_omega):
        F0 = F.softplus(self.raw_F0)
        A = F.softplus(self.raw_A)
        v0 = F.softplus(self.raw_v0) + 1e-6
        C1 = F.softplus(self.raw_C1)
        return F0 + A * (1.0 - torch.exp(-abs_omega / v0)) + C1 * abs_omega

    def export_curve(self, num_points=200):
        device = self.raw_F0.device
        x = torch.linspace(0.0, self.max_abs_omega, num_points, device=device)
        with torch.no_grad():
            y = self.forward(x)
        return x.cpu().numpy(), y.cpu().numpy()

    def export_parameters(self):
        with torch.no_grad():
            return {
                "F0": float(F.softplus(self.raw_F0).detach().cpu()),
                "A": float(F.softplus(self.raw_A).detach().cpu()),
                "v0": float(F.softplus(self.raw_v0).detach().cpu()),
                "C1": float(F.softplus(self.raw_C1).detach().cpu()),
            }


class FrictionEstimator(nn.Module):
    def __init__(
        self,
        J_nominal,
        max_abs_omega,
        B_init=0.0,
        learn_J=True,
        learn_B=True,
        use_B=True,
        scale_min=0.5,
        scale_max=1.5,
    ):
        super().__init__()
        self.J_nominal = float(J_nominal)
        self.scale_min = float(scale_min)
        self.scale_max = float(scale_max)
        self.learn_J = bool(learn_J)
        self.learn_B = bool(learn_B)
        self.use_B = bool(use_B)
        self.raw_scale = nn.Parameter(torch.tensor(0.0))
        self.raw_B = nn.Parameter(torch.tensor(float(B_init)))
        self.raw_omega_eps = nn.Parameter(torch.tensor(math.log(math.exp(0.05) - 1.0)))
        self.friction_model = FrictionMagnitudeModel(max_abs_omega=max_abs_omega)

    def J_scale(self):
        scale01 = torch.sigmoid(self.raw_scale)
        scale = self.scale_min + (self.scale_max - self.scale_min) * scale01
        if not self.learn_J:
            scale = scale.detach() * 0.0 + 1.0
        return scale

    def J_value(self):
        return self.J_nominal * self.J_scale()

    def B_value(self):
        if not self.use_B:
            return self.raw_B.detach() * 0.0
        if self.learn_B:
            return self.raw_B
        return self.raw_B.detach() * 0.0

    def omega_eps_value(self):
        return F.softplus(self.raw_omega_eps) + 1e-6

    def forward(self, omega, alpha):
        abs_omega = torch.abs(omega)
        sign_smooth = torch.tanh(omega / self.omega_eps_value())
        friction_abs = self.friction_model(abs_omega)
        friction_est = sign_smooth * friction_abs
        J = self.J_value()
        B = self.B_value()
        return friction_est, J, B

    def export_formula_parameters(self):
        params = self.friction_model.export_parameters()
        params["omega_eps"] = float(self.omega_eps_value().detach().cpu())
        return params


def symmetry_loss(abs_omega, omega, abs_friction, num_bins=20, min_samples=10):
    device = abs_omega.device
    max_speed = torch.max(abs_omega).detach()
    max_speed = torch.clamp(max_speed, min=1e-3)
    edges = torch.linspace(0.0, max_speed, num_bins + 1, device=device)
    losses = []
    for i in range(num_bins):
        lo = edges[i]
        hi = edges[i + 1]
        in_bin = (abs_omega >= lo) & (abs_omega < hi)
        pos = in_bin & (omega > 0)
        neg = in_bin & (omega < 0)
        if pos.sum() >= min_samples and neg.sum() >= min_samples:
            pos_mean = abs_friction[pos].median()
            neg_mean = abs_friction[neg].median()
            losses.append((pos_mean - neg_mean) ** 2)
    if not losses:
        return torch.tensor(0.0, device=device)
    return torch.stack(losses).mean()


def optimize_friction_pytorch(prepared, args):
    device = torch.device(
        "cuda" if args.device == "cuda" and torch.cuda.is_available() else "cpu"
    )
    print(f"Using PyTorch device: {device}")
    omega = torch.as_tensor(prepared["omega_used_rad_s"], dtype=torch.float32, device=device)
    alpha = torch.as_tensor(prepared["alpha_used_rad_s2"], dtype=torch.float32, device=device)
    torque = torch.as_tensor(prepared["tas_torque"], dtype=torch.float32, device=device)
    motor = torch.as_tensor(
        prepared["motor_contribution_0p5_AimiCurrent"], dtype=torch.float32, device=device
    )
    max_abs_omega = float(np.max(np.abs(prepared["omega_used_rad_s"])))
    estimator = FrictionEstimator(
        J_nominal=args.J_nominal,
        max_abs_omega=max_abs_omega,
        B_init=args.B_init,
        learn_J=args.learn_J,
        learn_B=args.learn_B,
        use_B=args.use_B,
    ).to(device)
    optimizer = torch.optim.Adam(estimator.parameters(), lr=args.lr)
    last_terms = {}
    history = []

    for epoch in range(1, args.epochs + 1):
        optimizer.zero_grad()
        friction_est, J, B = estimator(omega, alpha)
        residual = torque - motor - B * omega - J * alpha
        dyn_err = friction_est - residual
        L_dyn = F.huber_loss(friction_est, residual, reduction="mean", delta=0.2)
        abs_f = torch.abs(friction_est)
        if len(abs_f) > 1:
            abs_diff = abs_f[1:] - abs_f[:-1]
            jump = torch.abs(abs_diff)
            threshold = torch.quantile(jump.detach(), 0.95)
            L_smooth = torch.mean(abs_diff**2)
            L_spike = torch.mean(F.relu(jump - threshold) ** 2)
        else:
            L_smooth = torch.tensor(0.0, device=device)
            L_spike = torch.tensor(0.0, device=device)
        L_sym = symmetry_loss(torch.abs(omega), omega, abs_f)
        L_J = ((J - args.J_nominal) / max(args.J_nominal, 1e-9)) ** 2
        L_B = B**2 if args.use_B else torch.tensor(0.0, device=device)
        loss = (
            args.w_dyn * L_dyn
            + args.w_smooth * L_smooth
            + args.w_spike * L_spike
            + args.w_sym * L_sym
            + args.w_J * L_J
            + args.w_B * L_B
        )
        loss.backward()
        optimizer.step()

        last_terms = {
            "epoch": epoch,
            "loss": float(loss.detach().cpu()),
            "L_dyn": float(L_dyn.detach().cpu()),
            "L_smooth": float(L_smooth.detach().cpu()),
            "L_spike": float(L_spike.detach().cpu()),
            "L_sym": float(L_sym.detach().cpu()),
            "L_J": float(L_J.detach().cpu()),
            "L_B": float(L_B.detach().cpu()),
            "J": float(J.detach().cpu()),
            "J_scale": float(estimator.J_scale().detach().cpu()),
            "B": float(B.detach().cpu()),
            "threshold": float(threshold.detach().cpu()) if len(abs_f) > 1 else 0.0,
        }
        history.append(dict(last_terms))
        if epoch == 1 or epoch % 200 == 0 or epoch == args.epochs:
            print(
                f"epoch={epoch:4d} loss={last_terms['loss']:.6f} "
                f"L_dyn={last_terms['L_dyn']:.6f} "
                f"L_smooth={last_terms['L_smooth']:.6f} "
                f"L_spike={last_terms['L_spike']:.6f} "
                f"L_sym={last_terms['L_sym']:.6f} "
                f"J={last_terms['J']:.6f} "
                f"J_scale={last_terms['J_scale']:.4f} "
                f"B={last_terms['B']:.6f}"
            )

    with torch.no_grad():
        friction_learned, J_learned_tensor, B_learned_tensor = estimator(omega, alpha)
        residual_learned = torque - motor - B_learned_tensor * omega - J_learned_tensor * alpha
        curve_x, curve_y = estimator.friction_model.export_curve()
        formula_params = estimator.export_formula_parameters()

    print(
        "Learned analytic friction law: "
        "F(omega) = tanh(omega / omega_eps) * "
        "(F0 + A * (1 - exp(-|omega| / v0)) + C1 * |omega|)"
    )
    print(
        "  "
        f"omega_eps={formula_params['omega_eps']:.6f}, "
        f"F0={formula_params['F0']:.6f}, "
        f"A={formula_params['A']:.6f}, "
        f"v0={formula_params['v0']:.6f}, "
        f"C1={formula_params['C1']:.6f}"
    )

    return {
        "friction_learned": friction_learned.detach().cpu().numpy(),
        "dynamics_residual_learned": residual_learned.detach().cpu().numpy(),
        "J_learned": float(J_learned_tensor.detach().cpu()),
        "B_learned": float(B_learned_tensor.detach().cpu()),
        "J_scale": float(estimator.J_scale().detach().cpu()),
        "curve_abs_omega": curve_x,
        "curve_abs_friction": curve_y,
        "formula_params": formula_params,
        "loss_history": history,
        "loss_terms": last_terms,
    }


def compute_positive_negative_symmetry(abs_omega, abs_friction, omega, num_bins=20):
    edges, centers = build_speed_bins(abs_omega, num_bins=num_bins)
    pos_vals = []
    neg_vals = []
    counts_pos = []
    counts_neg = []
    for i in range(len(centers)):
        mask = (abs_omega >= edges[i]) & (abs_omega < edges[i + 1])
        pos = mask & (omega > 0)
        neg = mask & (omega < 0)
        pos_vals.append(np.median(abs_friction[pos]) if np.any(pos) else np.nan)
        neg_vals.append(np.median(abs_friction[neg]) if np.any(neg) else np.nan)
        counts_pos.append(int(np.sum(pos)))
        counts_neg.append(int(np.sum(neg)))
    return {
        "bin_centers": centers,
        "pos_median": np.asarray(pos_vals),
        "neg_median": np.asarray(neg_vals),
        "counts_pos": np.asarray(counts_pos),
        "counts_neg": np.asarray(counts_neg),
    }


def compute_diagnostics(prepared, friction_raw, friction_direct_smoothed, learned):
    omega = prepared["omega_used_rad_s"]
    alpha = prepared["alpha_used_rad_s2"]
    abs_raw = np.abs(friction_raw)
    abs_direct = np.abs(friction_direct_smoothed)
    abs_learned = np.abs(learned["friction_learned"])
    jump = np.abs(np.diff(abs_learned, prepend=abs_learned[0]))
    spike_threshold = np.quantile(jump, 0.95) if len(jump) > 0 else 0.0
    spike_mask = jump >= spike_threshold
    sign_omega = np.sign(omega)
    sign_alpha = np.sign(alpha)
    sign_disagree_rate = float(np.mean(sign_omega != sign_alpha))
    zero_speed_mask = np.abs(omega) < max(0.05, 0.05 * np.max(np.abs(omega)))
    symmetry = compute_positive_negative_symmetry(
        np.abs(omega), abs_learned, omega, num_bins=20
    )
    return {
        "abs_friction_raw": abs_raw,
        "abs_friction_direct_smoothed": abs_direct,
        "abs_friction_learned": abs_learned,
        "learned_jump": jump,
        "spike_threshold": float(spike_threshold),
        "spike_mask": spike_mask,
        "sign_omega": sign_omega,
        "sign_alpha": sign_alpha,
        "sign_disagree_rate": sign_disagree_rate,
        "zero_speed_spike_fraction": float(np.mean(spike_mask[zero_speed_mask])),
        "symmetry": symmetry,
    }


def save_results(prepared, friction_raw, friction_direct_smoothed, learned, args, diagnostics):
    ensure_parent_dir(args.output)
    out_df = pd.DataFrame(
        {
            "time": prepared["time"],
            "theta_rad": prepared["theta_rad"],
            "omega_measured_rad_s": prepared["omega_measured_rad_s"],
            "omega_from_angle_rad_s": prepared["omega_from_angle_rad_s"],
            "omega_used_rad_s": prepared["omega_used_rad_s"],
            "alpha_used_rad_s2": prepared["alpha_used_rad_s2"],
            "motor_input_AimiCurrent": prepared["motor_input_AimiCurrent"],
            "motor_contribution_0p5_AimiCurrent": prepared[
                "motor_contribution_0p5_AimiCurrent"
            ],
            "tas_torque": prepared["tas_torque"],
            "friction_raw": friction_raw,
            "friction_direct_smoothed": friction_direct_smoothed,
            "friction_learned": learned["friction_learned"],
            "abs_friction_raw": diagnostics["abs_friction_raw"],
            "abs_friction_direct_smoothed": diagnostics["abs_friction_direct_smoothed"],
            "abs_friction_learned": diagnostics["abs_friction_learned"],
            "dynamics_residual_learned": learned["dynamics_residual_learned"],
            "J_nominal": np.full_like(prepared["time"], args.J_nominal, dtype=float),
            "J_learned": np.full_like(prepared["time"], learned["J_learned"], dtype=float),
            "B_learned": np.full_like(prepared["time"], learned["B_learned"], dtype=float),
        }
    )
    out_df.to_csv(args.output, index=False)
    summary_path, loss_history_path = derive_aux_output_paths(args.output)
    loss_history = learned.get("loss_history", [])
    if loss_history:
        ensure_parent_dir(loss_history_path)
        pd.DataFrame(loss_history).to_csv(loss_history_path, index=False)
    summary = {
        "data_file": args.data,
        "run_name": args.run_name,
        "run_dir": getattr(args, "run_dir", None),
        "mode": args.mode,
        "omega_source": args.omega_source,
        "sample_time_used": prepared["Ts"],
        "time_step_estimated_from_data": prepared["dt_est"],
        "derivative_filter_time": args.derivative_filter_time,
        "J_nominal": args.J_nominal,
        "J_learned": learned["J_learned"],
        "J_scale": learned["J_scale"],
        "B_learned": learned["B_learned"],
        "analytic_formula": "tanh(omega / omega_eps) * (F0 + A * (1 - exp(-|omega| / v0)) + C1 * |omega|)",
        "analytic_parameters": learned.get("formula_params", {}),
        "loss_terms": learned["loss_terms"],
        "sign_disagree_rate": diagnostics["sign_disagree_rate"],
        "zero_speed_spike_fraction": diagnostics["zero_speed_spike_fraction"],
        "notes": [
            "AimiCurrent[] is interpreted as the Simulink input before the 0.5 gain.",
            "Friction sign is modeled from steering velocity, not angular acceleration.",
            "J_nominal is a regularization center and sanity check, not a verified runtime value.",
        ],
    }
    if not args.use_B:
        summary["notes"].append("B term disabled: viscous damping contribution forced to zero.")
    ensure_parent_dir(summary_path)
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"Saved results CSV to {args.output}")
    if loss_history:
        print(f"Saved loss history to {loss_history_path}")
    print(f"Saved fit summary to {summary_path}")


def plot_results(prepared, friction_raw, friction_direct_smoothed, learned, diagnostics, args):
    os.makedirs(args.plot_dir, exist_ok=True)
    time = prepared["time"]
    omega_used = prepared["omega_used_rad_s"]
    alpha_used = prepared["alpha_used_rad_s2"]
    theta = prepared["theta_rad"]

    def savefig(name):
        path = os.path.join(args.plot_dir, name)
        plt.tight_layout()
        plt.savefig(path, dpi=150)
        plt.close()

    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time, theta, label="theta_rad")
    plt.ylabel("rad")
    plt.title("Angle, Omega, Alpha")
    plt.grid(True)
    plt.legend()
    plt.subplot(3, 1, 2)
    plt.plot(time, omega_used, label="omega_used")
    plt.ylabel("rad/s")
    plt.grid(True)
    plt.legend()
    plt.subplot(3, 1, 3)
    plt.plot(time, alpha_used, label="alpha_used")
    plt.xlabel("Time [s]")
    plt.ylabel("rad/s^2")
    plt.grid(True)
    plt.legend()
    savefig("angle_omega_alpha.png")

    omega_error = prepared["omega_measured_rad_s"] - prepared["omega_from_angle_rad_s"]
    plt.figure(figsize=(12, 7))
    plt.subplot(2, 1, 1)
    plt.plot(time, prepared["omega_measured_rad_s"], label="omega_measured")
    plt.plot(time, prepared["omega_from_angle_rad_s"], label="omega_from_angle", alpha=0.8)
    plt.ylabel("rad/s")
    plt.title("Measured vs Angle-Derived Omega")
    plt.grid(True)
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(time, omega_error, label="omega_error")
    plt.xlabel("Time [s]")
    plt.ylabel("rad/s")
    plt.grid(True)
    plt.legend()
    savefig("omega_comparison.png")

    plt.figure(figsize=(12, 5))
    plt.plot(time, friction_raw, label="friction_raw")
    plt.xlabel("Time [s]")
    plt.ylabel("Nm")
    plt.title("Raw Friction Residual")
    plt.grid(True)
    plt.legend()
    savefig("raw_residual.png")

    plt.figure(figsize=(12, 5))
    plt.plot(time, friction_raw, label="raw", alpha=0.5)
    plt.plot(time, friction_direct_smoothed, label="direct_smoothed", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Nm")
    plt.title("Direct Smoothed Friction")
    plt.grid(True)
    plt.legend()
    savefig("direct_smoothed_friction.png")

    plt.figure(figsize=(12, 5))
    plt.plot(time, friction_raw, label="raw", alpha=0.35)
    plt.plot(time, friction_direct_smoothed, label="direct_smoothed", linewidth=2)
    plt.plot(time, learned["friction_learned"], label="learned", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Nm")
    plt.title("Learned Friction Comparison")
    plt.grid(True)
    plt.legend()
    savefig("learned_friction.png")

    plt.figure(figsize=(12, 5))
    plt.plot(time, diagnostics["abs_friction_raw"], label="abs_raw", alpha=0.35)
    plt.plot(time, diagnostics["abs_friction_direct_smoothed"], label="abs_direct", linewidth=2)
    plt.plot(time, diagnostics["abs_friction_learned"], label="abs_learned", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("|Friction| [Nm]")
    plt.title("Absolute Friction vs Time")
    plt.grid(True)
    plt.legend()
    savefig("abs_friction_time.png")

    plt.figure(figsize=(12, 6))
    plt.scatter(omega_used, friction_raw, s=6, alpha=0.2, label="raw")
    plt.scatter(omega_used, friction_direct_smoothed, s=6, alpha=0.25, label="direct_smoothed")
    plt.scatter(omega_used, learned["friction_learned"], s=6, alpha=0.25, label="learned")
    plt.xlabel("Omega [rad/s]")
    plt.ylabel("Friction [Nm]")
    plt.title("Friction vs Omega")
    plt.grid(True)
    plt.legend()
    savefig("friction_vs_omega.png")

    plt.figure(figsize=(12, 6))
    plt.scatter(np.abs(omega_used), diagnostics["abs_friction_raw"], s=6, alpha=0.15, label="abs_raw")
    plt.scatter(
        np.abs(omega_used),
        diagnostics["abs_friction_direct_smoothed"],
        s=6,
        alpha=0.2,
        label="abs_direct",
    )
    plt.scatter(
        np.abs(omega_used),
        diagnostics["abs_friction_learned"],
        s=6,
        alpha=0.2,
        label="abs_learned",
    )
    plt.plot(
        learned["curve_abs_omega"],
        learned["curve_abs_friction"],
        color="black",
        linewidth=2,
        label="learned_trend",
    )
    plt.xlabel("|Omega| [rad/s]")
    plt.ylabel("|Friction| [Nm]")
    plt.title("Absolute Friction vs Absolute Omega")
    plt.grid(True)
    plt.legend()
    savefig("abs_friction_vs_abs_omega.png")

    sym = diagnostics["symmetry"]
    plt.figure(figsize=(12, 5))
    plt.plot(sym["bin_centers"], sym["pos_median"], marker="o", label="omega > 0")
    plt.plot(sym["bin_centers"], sym["neg_median"], marker="o", label="omega < 0")
    plt.xlabel("|Omega| [rad/s]")
    plt.ylabel("Median |Friction| [Nm]")
    plt.title("Positive / Negative Symmetry")
    plt.grid(True)
    plt.legend()
    savefig("positive_negative_symmetry.png")

    plt.figure(figsize=(12, 6))
    plt.plot(time, diagnostics["learned_jump"], label="|diff(abs(friction_learned))|")
    plt.axhline(
        diagnostics["spike_threshold"], color="red", linestyle="--", label="95% threshold"
    )
    spike_times = time[diagnostics["spike_mask"]]
    spike_vals = diagnostics["learned_jump"][diagnostics["spike_mask"]]
    plt.scatter(spike_times, spike_vals, color="red", s=10, label="detected spikes")
    plt.xlabel("Time [s]")
    plt.ylabel("Jump")
    plt.title("Spike Diagnostic")
    plt.grid(True)
    plt.legend()
    savefig("spike_diagnostic.png")

    plt.figure(figsize=(12, 5))
    plt.plot(time, learned["friction_learned"], label="learned_friction")
    plt.plot(time, learned["dynamics_residual_learned"], label="learned_residual", alpha=0.8)
    plt.xlabel("Time [s]")
    plt.ylabel("Nm")
    plt.title("Learned Friction vs Learned Dynamics Residual")
    plt.grid(True)
    plt.legend()
    savefig("residual_comparison.png")

    plt.figure(figsize=(12, 6))
    plt.plot(time, diagnostics["sign_omega"], label="sign(omega)")
    plt.plot(time, diagnostics["sign_alpha"], label="sign(alpha)", alpha=0.8)
    plt.xlabel("Time [s]")
    plt.ylabel("Sign")
    plt.title("Sign Diagnostic")
    plt.grid(True)
    plt.legend()
    savefig("sign_diagnostic.png")

    plt.figure(figsize=(8, 5))
    fp = learned.get("formula_params", {})
    labels = ["J_nominal", "J_learned", "J_scale", "B_learned", "F0", "A", "v0", "C1", "omega_eps"]
    values = [
        args.J_nominal,
        learned["J_learned"],
        learned["J_scale"],
        learned["B_learned"],
        fp.get("F0", float("nan")),
        fp.get("A", float("nan")),
        fp.get("v0", float("nan")),
        fp.get("C1", float("nan")),
        fp.get("omega_eps", float("nan")),
    ]
    plt.bar(labels, values)
    plt.title("Parameter Summary")
    plt.grid(True, axis="y")
    plt.xticks(rotation=30, ha="right")
    savefig("parameter_summary.png")

    loss_history = learned.get("loss_history", [])
    if loss_history:
        hist = pd.DataFrame(loss_history)
        plt.figure(figsize=(12, 6))
        plt.plot(hist["epoch"], hist["loss"], label="loss", linewidth=2)
        if "L_dyn" in hist:
            plt.plot(hist["epoch"], hist["L_dyn"], label="L_dyn", alpha=0.9)
        if "L_smooth" in hist:
            plt.plot(hist["epoch"], hist["L_smooth"], label="L_smooth", alpha=0.9)
        if "L_spike" in hist:
            plt.plot(hist["epoch"], hist["L_spike"], label="L_spike", alpha=0.9)
        if "L_sym" in hist:
            plt.plot(hist["epoch"], hist["L_sym"], label="L_sym", alpha=0.9)
        if "L_J" in hist:
            plt.plot(hist["epoch"], hist["L_J"], label="L_J", alpha=0.9)
        if "L_B" in hist:
            plt.plot(hist["epoch"], hist["L_B"], label="L_B", alpha=0.9)
        plt.xlabel("Epoch")
        plt.ylabel("Loss")
        plt.title("Loss Curve")
        plt.grid(True)
        plt.legend()
        savefig("loss_curve.png")
    print(f"Saved plots to {args.plot_dir}")


def build_parser():
    parser = argparse.ArgumentParser(description="Estimate EPS steering friction from SWA logs.")
    parser.add_argument("--data", type=str, default=None)
    parser.add_argument("--mode", choices=["direct", "learn", "both"], default="both")
    parser.add_argument("--angle-unit", choices=["deg", "rad"], default="deg")
    parser.add_argument("--time-col", type=str, default=DEFAULT_TIME_COL)
    parser.add_argument("--torque-col", type=str, default=DEFAULT_TORQUE_COL)
    parser.add_argument("--angle-col", type=str, default=DEFAULT_ANGLE_COL)
    parser.add_argument("--omega-col", type=str, default=DEFAULT_OMEGA_COL)
    parser.add_argument("--motor-input-col", type=str, default=DEFAULT_MOTOR_INPUT_COL)
    parser.add_argument("--omega-source", choices=["measured", "angle"], default="measured")
    parser.add_argument("--derivative-filter-time", type=float, default=0.1)
    parser.add_argument("--J-nominal", type=float, default=0.01275)
    parser.add_argument("--B-init", type=float, default=0.0)
    parser.add_argument("--use-B", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--learn-J", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--learn-B", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--epochs", type=int, default=3000)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--w-dyn", type=float, default=1.0)
    parser.add_argument("--w-smooth", type=float, default=0.1)
    parser.add_argument("--w-spike", type=float, default=0.1)
    parser.add_argument("--w-sym", type=float, default=0.1)
    parser.add_argument("--w-J", type=float, default=1.0)
    parser.add_argument("--w-B", type=float, default=0.01)
    parser.add_argument("--run-name", type=str, default=None)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--plot-dir", type=str, default=None)
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
    return parser


def main():
    args = build_parser().parse_args()
    args.run_name, args.run_dir, args.output, args.plot_dir = resolve_output_layout(args)
    print(f"Run name: {args.run_name}")
    print(f"Run directory: {args.run_dir}")
    J_geom = compute_nominal_inertia()
    if abs(J_geom - args.J_nominal) / max(args.J_nominal, 1e-9) > 0.5:
        print(
            "Note: geometry-based inertia estimate differs from the default J_nominal. "
            "Using CLI/default J_nominal as the regularization center unless overridden."
        )
    else:
        print("Geometry-based inertia estimate is reasonably close to the default J_nominal.")
    print(
        "Diagnostic note: the script does not use sign(alpha) as the friction sign because "
        "friction direction is more physically tied to velocity direction."
    )

    if args.data:
        try:
            df = load_data(args.data)
        except RuntimeError as exc:
            print(str(exc), file=sys.stderr)
            return 1
        expected_cols = [
            args.time_col or DEFAULT_TIME_COL,
            args.torque_col or DEFAULT_TORQUE_COL,
            args.angle_col or DEFAULT_ANGLE_COL,
            args.omega_col or DEFAULT_OMEGA_COL,
            args.motor_input_col or DEFAULT_MOTOR_INPUT_COL,
        ]
        normalized_loaded_cols = {_normalize_col_name(col) for col in df.columns}
        if not {_normalize_col_name(col) for col in expected_cols}.issubset(normalized_loaded_cols):
            header_row, df_detected = find_header_row(args.data, expected_cols, max_scan_rows=20)
            if df_detected is not None:
                print(
                    f"Auto-detected Excel header row at 0-based row {header_row} "
                    f"(Excel line {header_row + 1})."
                )
                df = df_detected
        print(f"Loaded data from {args.data} with shape {df.shape}")
    else:
        df = generate_synthetic_data(sample_time=FIXED_FILTER_TS)
        args.data = "synthetic_demo"

    try:
        columns = detect_columns(df, args)
    except KeyError as exc:
        print(str(exc), file=sys.stderr)
        if args.data and args.data != "synthetic_demo":
            print(
                "Tip: this file may contain metadata rows before the real column header. "
                "The script now scans for that automatically, but if the format changes further, "
                "inspect the first few rows in Excel and pass explicit column names if needed.",
                file=sys.stderr,
            )
        return 1
    df = clean_numeric_data(df, columns)
    prepared = prepare_signals(df, columns, args)
    smoothing_config = SmoothingConfig()
    B_direct = args.B_init if args.use_B else 0.0
    friction_raw, friction_direct_smoothed = direct_friction_estimate(
        prepared, J=args.J_nominal, B=B_direct, smoothing_config=smoothing_config
    )

    if args.mode == "direct":
        learned = {
            "friction_learned": friction_direct_smoothed.copy(),
            "dynamics_residual_learned": friction_raw.copy(),
            "J_learned": args.J_nominal,
            "B_learned": B_direct,
            "J_scale": 1.0,
            "curve_abs_omega": np.linspace(0.0, np.max(np.abs(prepared["omega_used_rad_s"])), 200),
            "curve_abs_friction": np.full(200, np.nanmedian(np.abs(friction_direct_smoothed))),
            "formula_params": {
                "omega_eps": 0.05,
                "F0": float(np.nanmedian(np.abs(friction_direct_smoothed))),
                "A": 0.0,
                "v0": 1.0,
                "C1": 0.0,
            },
            "loss_terms": {},
        }
    else:
        learned = optimize_friction_pytorch(prepared, args)
        if args.mode == "learn":
            friction_direct_smoothed = robust_smooth(friction_raw, smoothing_config, prepared["Ts"])

    diagnostics = compute_diagnostics(prepared, friction_raw, friction_direct_smoothed, learned)
    print(f"Sign disagreement rate sign(omega) vs sign(alpha): {diagnostics['sign_disagree_rate']:.3f}")
    print(
        f"Fraction of detected learned spikes near low speed: "
        f"{diagnostics['zero_speed_spike_fraction']:.3f}"
    )
    save_results(prepared, friction_raw, friction_direct_smoothed, learned, args, diagnostics)
    plot_results(prepared, friction_raw, friction_direct_smoothed, learned, diagnostics, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
