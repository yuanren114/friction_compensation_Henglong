import argparse
import math
import os
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd


DEFAULTS = {
    "J": 0.01275,
    "A": 0.0016515420284122229,
    "v0": 4.297909736633301,
    "C1": 0.04468029364943504,
    "omega_eps": 0.003496480407193303,
    "Ts": 0.05,
    "Tfilter": 0.25,
    "F0_init": 0.4111127257347107,
    "mu_F0": 1e-2,
    "F0_min": 0.05,
    "F0_max": 2.0,
    "omega_min": 0.05,
    "alpha_max": 20.0,
    "e_clip": 1.0,
    "e_high": 2.0,
    "e_low": 0.2,
    "recover_needed": 5,
    "window_duration": 5.0,
    "min_valid_count": 10,
}


@dataclass
class SimState:
    x_prev: float = None
    e_hold: float = 0.0
    freeze_state: bool = False
    recover_count: int = 0
    win_elapsed: float = 0.0
    sum_mag: float = 0.0
    count_valid: int = 0
    t_prev: float = None
    F0: float = DEFAULTS["F0_init"]


def detect_header_row(path, target_names, max_scan_rows=10):
    for header in range(max_scan_rows + 1):
        try:
            df = read_excel_auto(path, header=header)
        except Exception:
            continue
        cols = {str(c).strip() for c in df.columns}
        if all(name in cols for name in target_names):
            return header
    return 0


def read_excel_auto(path, **kwargs):
    ext = os.path.splitext(path)[1].lower()
    engines = []
    if ext == ".xls":
        engines = ["xlrd", "openpyxl"]
    else:
        engines = ["openpyxl", "xlrd"]
    last_exc = None
    for engine in engines:
        try:
            return pd.read_excel(path, engine=engine, **kwargs)
        except Exception as exc:
            last_exc = exc
    if last_exc is not None:
        # Some files in this workspace use SpreadsheetML 2003 XML with a .xls extension.
        if ext == ".xls":
            try:
                return read_spreadsheetml_2003(path)
            except Exception:
                pass
        raise last_exc
    return pd.read_excel(path, **kwargs)


def read_spreadsheetml_2003(path):
    ns = {
        "ss": "urn:schemas-microsoft-com:office:spreadsheet",
    }
    tree = ET.parse(path)
    root = tree.getroot()
    table = root.find(".//ss:Worksheet/ss:Table", ns)
    if table is None:
        raise ValueError("No SpreadsheetML table found")

    rows = []
    max_cols = 0
    for row in table.findall("ss:Row", ns):
        values = []
        current_col = 1
        for cell in row.findall("ss:Cell", ns):
            idx = cell.attrib.get("{urn:schemas-microsoft-com:office:spreadsheet}Index")
            if idx is not None:
                idx = int(idx)
                while current_col < idx:
                    values.append(None)
                    current_col += 1
            data = cell.find("ss:Data", ns)
            values.append(data.text if data is not None else None)
            current_col += 1
        max_cols = max(max_cols, len(values))
        rows.append(values)

    padded = [r + [None] * (max_cols - len(r)) for r in rows]
    if not padded:
        return pd.DataFrame()
    header = padded[0]
    body = padded[1:]
    cols = []
    for i, c in enumerate(header):
        cols.append(str(c) if c not in (None, "") else f"Unnamed: {i}")
    return pd.DataFrame(body, columns=cols)


def read_spreadsheetml_with_header(path, header_row=6):
    raw = read_spreadsheetml_2003(path)
    if raw.empty:
        return raw
    cols = []
    for i, c in enumerate(raw.iloc[header_row].tolist()):
        cols.append(str(c) if c not in (None, "", "None") else f"Unnamed: {i}")
    body = raw.iloc[header_row + 1 :].reset_index(drop=True).copy()
    body.columns = cols
    return body


def load_xls(path):
    stem = os.path.basename(path).lower()
    if stem.startswith("f0_increase_demo_"):
        df = read_spreadsheetml_with_header(path, header_row=5)
        mapping = {
            "time": "t[s]",
            "omega_deg_s": "HwAngVel_Degs_s32p16[]",
            "torque_hand": "HwTrq_Nm_s16p10[]",
            "aimi": "AimiCurrent[]",
        }
        return df, mapping

    # FC_constant and SWA-style fallback
    candidates = [
        "t[s]",
        "HwAngVel_Degs_s32p16[]",
        "HwTrq_Nm_s16p10[]",
        "AimiCurrent[]",
    ]
    header = detect_header_row(path, candidates, max_scan_rows=10)
    df = read_excel_auto(path, header=header)
    mapping = {
        "time": "t[s]",
        "omega_deg_s": "HwAngVel_Degs_s32p16[]",
        "torque_hand": "HwTrq_Nm_s16p10[]",
        "aimi": "AimiCurrent[]",
    }
    # SWA-style long names
    if "API_2ms.HwAngVel_Degs_s32p16[]" in df.columns:
        mapping["omega_deg_s"] = "API_2ms.HwAngVel_Degs_s32p16[]"
    if "API_2ms.HwTrq_Nm_s16p10[]" in df.columns:
        mapping["torque_hand"] = "API_2ms.HwTrq_Nm_s16p10[]"
    return df, mapping


def filtered_derivative_step(omega, Ts, Tfilter, state: SimState):
    if state.x_prev is None:
        state.x_prev = omega
    a = Ts / max(Tfilter, 1e-6)
    a = min(max(a, 0.0), 1.0)
    alpha = (omega - state.x_prev) / max(Tfilter, 1e-6)
    x_new = a * omega + (1.0 - a) * state.x_prev
    state.x_prev = x_new
    return alpha


def friction_hat(omega, F0, A, v0, C1, omega_eps):
    absw = abs(omega)
    s = math.tanh(omega / max(omega_eps, 1e-6))
    phi = 1.0 - math.exp(-absw / max(v0, 1e-6))
    mag = max(0.0, F0 + A * phi + C1 * absw)
    return s * mag


def error_preprocess_step(residual, fhat, omega, alpha, t_now, p, state: SimState):
    if state.t_prev is None:
        state.t_prev = t_now
    dt = t_now - state.t_prev
    if dt < 0.0:
        dt = 0.0
    state.t_prev = t_now

    e_raw = residual - fhat
    e_mag = abs(residual) - abs(fhat)
    trusted_sample = (abs(omega) > p["omega_min"]) and (abs(alpha) < p["alpha_max"])
    anomaly = (abs(e_raw) > p["e_high"]) or (abs(alpha) > p["alpha_max"])
    recovered = trusted_sample and (abs(e_raw) < p["e_low"])

    if (not state.freeze_state) and anomaly:
        state.freeze_state = True
        state.recover_count = 0
    elif state.freeze_state:
        if recovered:
            state.recover_count += 1
            if state.recover_count >= int(p["recover_needed"]):
                state.freeze_state = False
                state.recover_count = 0
        else:
            state.recover_count = 0

    sample_ok = trusted_sample and (not state.freeze_state)
    e_used = max(-p["e_clip"], min(p["e_clip"], e_mag))
    update_enable = False
    state.win_elapsed += dt
    if sample_ok:
        state.sum_mag += e_used
        state.count_valid += 1

    if state.win_elapsed >= max(p["window_duration"], 1e-6):
        if state.count_valid >= max(1, round(p["min_valid_count"])):
            state.e_hold = state.sum_mag / state.count_valid
            update_enable = True
        state.win_elapsed = 0.0
        state.sum_mag = 0.0
        state.count_valid = 0

    return {
        "e_raw": e_raw,
        "e_slow": state.e_hold,
        "update_enable": update_enable,
        "freeze_active": state.freeze_state,
        "count_valid_out": state.count_valid,
        "win_elapsed_out": state.win_elapsed,
        "trusted_sample": trusted_sample,
        "sample_ok": sample_ok,
        "anomaly": anomaly,
        "recovered": recovered,
    }


def update_f0_step(e_slow, update_enable, state: SimState, p):
    F0_next = state.F0
    if update_enable:
        F0_next = state.F0 + p["mu_F0"] * e_slow
    F0_next = min(max(F0_next, p["F0_min"]), p["F0_max"])
    changed = F0_next != state.F0
    state.F0 = F0_next
    return changed


def simulate(df, mapping, params):
    state = SimState(F0=params["F0_init"])
    rows = []
    updates = []
    t = pd.to_numeric(df[mapping["time"]], errors="coerce").to_numpy(dtype=float)
    omega_deg_s = pd.to_numeric(df[mapping["omega_deg_s"]], errors="coerce").to_numpy(dtype=float)
    torque_hand = pd.to_numeric(df[mapping["torque_hand"]], errors="coerce").to_numpy(dtype=float)
    aimi = pd.to_numeric(df[mapping["aimi"]], errors="coerce").to_numpy(dtype=float)

    mask = np.isfinite(t) & np.isfinite(omega_deg_s) & np.isfinite(torque_hand) & np.isfinite(aimi)
    t = t[mask]
    omega_deg_s = omega_deg_s[mask]
    torque_hand = torque_hand[mask]
    aimi = aimi[mask]

    for i in range(len(t)):
        omega = math.radians(omega_deg_s[i])
        alpha = filtered_derivative_step(omega, params["Ts"], params["Tfilter"], state)
        residual = torque_hand[i] - 0.5 * aimi[i] - params["J"] * alpha
        fhat = friction_hat(omega, state.F0, params["A"], params["v0"], params["C1"], params["omega_eps"])
        e = error_preprocess_step(residual, fhat, omega, alpha, t[i], params, state)
        f0_before = state.F0
        changed = update_f0_step(e["e_slow"], e["update_enable"], state, params)
        if e["update_enable"]:
            updates.append({
                "t": t[i],
                "F0_before": f0_before,
                "e_slow": e["e_slow"],
                "F0_after": state.F0,
                "changed": changed,
            })
        rows.append({
            "t": t[i],
            "omega_rad_s": omega,
            "alpha_rad_s2": alpha,
            "residual": residual,
            "friction_hat": fhat,
            "F0": state.F0,
            **e,
        })
    return pd.DataFrame(rows), pd.DataFrame(updates)


def print_summary(df_sim, df_updates, params, path):
    print(f"File: {path}")
    if len(df_sim) == 0:
        print("No valid rows.")
        return
    print(f"Rows: {len(df_sim)}")
    print(f"Time span: {df_sim['t'].iloc[0]:.6f} -> {df_sim['t'].iloc[-1]:.6f} s")
    print(f"Final F0: {df_sim['F0'].iloc[-1]:.9f}")
    print(f"F0 min/max: {df_sim['F0'].min():.9f} / {df_sim['F0'].max():.9f}")
    print(f"Any update_enable pulse: {bool(df_sim['update_enable'].any())}")
    print(f"Num update_enable pulses: {int(df_sim['update_enable'].sum())}")
    print(f"Num actual F0 changes: {int((df_sim['F0'].diff().fillna(0) != 0).sum())}")
    print(f"Trusted sample ratio: {df_sim['trusted_sample'].mean():.3f}")
    print(f"Sample OK ratio: {df_sim['sample_ok'].mean():.3f}")
    print(f"Freeze active ratio: {df_sim['freeze_active'].mean():.3f}")
    print(f"Anomaly ratio: {df_sim['anomaly'].mean():.3f}")
    print(f"Max win_elapsed_out: {df_sim['win_elapsed_out'].max():.6f}")
    print(f"Max count_valid_out: {df_sim['count_valid_out'].max():.0f}")
    if len(df_updates):
        print("\nWindow updates:")
        print(df_updates.head(20).to_string(index=False))
    else:
        print("\nNo window updates were triggered.")
        tail = df_sim[["t", "omega_rad_s", "alpha_rad_s2", "residual", "friction_hat",
                      "e_raw", "e_slow", "trusted_sample", "sample_ok",
                      "freeze_active", "count_valid_out", "win_elapsed_out"]].tail(10)
        print(tail.to_string(index=False))


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--Ts", type=float, default=DEFAULTS["Ts"])
    ap.add_argument("--Tfilter", type=float, default=DEFAULTS["Tfilter"])
    ap.add_argument("--window-duration", type=float, default=DEFAULTS["window_duration"])
    ap.add_argument("--min-valid-count", type=int, default=DEFAULTS["min_valid_count"])
    ap.add_argument("--csv-out", default="")
    ap.add_argument("--updates-out", default="")
    return ap.parse_args()


def main():
    args = parse_args()
    params = dict(DEFAULTS)
    params["Ts"] = args.Ts
    params["Tfilter"] = args.Tfilter
    params["window_duration"] = args.window_duration
    params["min_valid_count"] = args.min_valid_count

    df, mapping = load_xls(args.data)
    df_sim, df_updates = simulate(df, mapping, params)
    print_summary(df_sim, df_updates, params, args.data)

    if args.csv_out:
        df_sim.to_csv(args.csv_out, index=False)
        print(f"Saved sim trace: {args.csv_out}")
    if args.updates_out:
        df_updates.to_csv(args.updates_out, index=False)
        print(f"Saved window updates: {args.updates_out}")


if __name__ == "__main__":
    main()
