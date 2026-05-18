# friction_estimator cheatsheet

## Purpose

`friction_estimator.py` estimates EPS steering friction from logged steering data.

Current model used in the script:

```text
friction_raw
= TAS_Torque
- 0.5 * AimiCurrent
- B * omega
- J * alpha
```

Where:

- `TAS_Torque` comes from `API_2ms.HwTrq_Nm_s16p10[]`
- `AimiCurrent[]` is treated as the Simulink-side input before the `0.5` gain
- `omega` is steering angular velocity in `rad/s`
- `alpha` is steering angular acceleration in `rad/s^2`
- `B` is viscous damping
- `J` is equivalent inertia

The derivative block in the script currently uses:

- fixed `Ts = 0.1`
- user-provided `T` through `--derivative-filter-time`

The learned friction law is now analytic, not binned:

```text
F_fric(omega) = tanh(omega / omega_eps) * (F0 + A * (1 - exp(-|omega| / v0)) + C1 * |omega|)
```

Where:

- `omega_eps` controls the smooth sign transition near zero speed
- `F0` is the base friction level
- `A` and `v0` control the low-speed exponential rise / transition
- `C1` controls linear growth with `|omega|`

## Most Important Commands

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --omega-source measured --derivative-filter-time 0.5
```

Use this first. Runs both the direct residual estimate and the PyTorch learned estimate on the real Excel file.
By default, this now creates an auto-named run folder under `outputs/runs/`.

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --omega-source measured --derivative-filter-time 0.5 --no-use-B
```

Use this when you do not want to assume viscous friction. The `B * omega` term is removed completely.

```bash
python friction_estimator.py --data SWA.XLS --mode direct --angle-unit deg --omega-source measured --derivative-filter-time 0.5
```

Use this for a fast physics-only residual estimate without PyTorch learning.

```bash
python friction_estimator.py --data SWA.XLS --mode learn --angle-unit deg --omega-source measured --derivative-filter-time 0.5 --epochs 3000 --lr 1e-3
```

Use this when you want only the learned friction model and do not need the direct-mode result as the main focus.

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --omega-source measured --derivative-filter-time 1.0 --no-use-B
```

Use this when you want a more conservative derivative estimate and no viscous damping term.

```bash
python friction_estimator.py --mode both --angle-unit deg --omega-source measured --derivative-filter-time 0.5
```

Use this to run the synthetic demo dataset when you do not want to load an Excel file.

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --omega-source measured --derivative-filter-time 0.5 --device cpu
```

Use this when you want to avoid CUDA and force CPU execution.

## Full Default Command

This is a full explicit command with the current script defaults written out:

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --time-col "t[s]" --torque-col "API_2ms.HwTrq_Nm_s16p10[]" --angle-col "API_2ms.HwAng_Deg_s32p16[]" --omega-col "API_2ms.HwAngVel_Degs_s32p16[]" --motor-input-col "AimiCurrent[]" --omega-source measured --derivative-filter-time 0.1 --J-nominal 0.01275 --B-init 0.0 --init-F0 1.0 --init-A 0.05 --init-v0 0.5 --init-C1 0.01 --init-omega-eps 0.05 --use-B --learn-J --learn-B --epochs 3000 --lr 1e-3 --w-dyn 1.0 --w-smooth 0.1 --w-spike 0.1 --w-sym 0.1 --w-J 1.0 --w-B 0.01 --device cuda
```

Notes:

- the derivative block still uses fixed `Ts = 0.1` internally
- the command above shows the script defaults, not the recommended tuned final settings
- for your current preferred analytic run, you are using a more specific command near the top of this file, such as `--derivative-filter-time 0.3` and `--no-use-B`
- if `--output` and `--plot-dir` are omitted, the script auto-creates:
  - `outputs/runs/<run_name>/results_friction.csv`
  - `outputs/runs/<run_name>/results_friction_summary.json`
  - `outputs/runs/<run_name>/results_friction_loss_history.csv`
  - `outputs/runs/<run_name>/plots/`

## Auto Output Layout

Default behavior is now fully automatic per run.

If you do not provide `--output` or `--plot-dir`, the script creates:

```text
outputs/runs/<run_name>/
  results_friction.csv
  results_friction_summary.json
  results_friction_loss_history.csv
  plots/
```

You can optionally set:

```bash
--run-name my_experiment_name
```

Example:

```bash
python friction_estimator.py --data SWA.XLS --run-name my_test
```

This writes outputs to:

```text
outputs/runs/my_test/
```

## Best Runs

### Best Pre-Analytic Run

This was the best version before the script was changed to the analytic friction law:

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --omega-source measured --derivative-filter-time 0.3 --no-use-B --device cpu --epochs 2000 --w-smooth 0.5 --w-spike 0.3 --w-sym 0.2 --output tuned_final.csv --plot-dir tuned_final_plots
```

This corresponds to the older non-analytic learned friction version.

### Best Current Analytic Run

This is the current preferred analytic version:

```bash
python friction_estimator.py --data SWA.XLS --mode both --angle-unit deg --omega-source measured --derivative-filter-time 0.3 --no-use-B --device cuda --epochs 2000 --output tuned_final_analytic.csv --plot-dir tuned_final_analytic_plots
```

Reasoning:

- `T = 0.3` gave a reasonable derivative response for this log
- `--no-use-B` avoids assuming viscous friction when it is not clearly justified
- `cuda` is fine for this version if your environment is stable

The corresponding learned analytic formula from the current saved result is:

```text
F_fric(omega) = tanh(omega / omega_eps) * (F0 + A * (1 - exp(-|omega| / v0)) + C1 * |omega|)
```

Saved parameter values:

```text
omega_eps = 0.011560
F0        = 0.454762
A         = 0.014245
v0        = 0.077815
C1        = 0.038584
J         = 0.012759
B         = 0
```

## Final Result Files

For the current analytic version, the main final outputs are:

- `tuned_final_analytic.csv`
- `tuned_final_analytic_plots/`
- `friction_fit_summary.json`
- `loss_history.csv`

These filenames describe the historical runs already saved in the workspace.
For new runs, the default output location is the auto-generated run folder under `outputs/runs/`.

`friction_fit_summary.json` contains the learned analytic formula parameters:

- `omega_eps`
- `F0`
- `A`
- `v0`
- `C1`

## Outputs

Main outputs:

- `results_friction.csv`
- `friction_fit_summary.json`
- plot folder, default `friction_plots/`
- `loss_history.csv`

If you run with custom names such as `tuned_final_analytic.csv`, those custom names become the main result files for that run.

Loss outputs:

- `loss_history.csv`: one row per epoch with `loss`, `L_dyn`, `L_smooth`, `L_spike`, `L_sym`, `L_J`, `L_B`, `J`, `J_scale`, `B`
- `loss_curve.png`: saved inside the selected plot directory

## Argument Reference

### `--data`

Path to input data file.

- Supports `.xls` and `.xlsx`
- If omitted, the script generates synthetic demo data
- Example:

```bash
--data SWA.XLS
```

### `--mode`

Controls which estimation path is used.

Options:

- `direct`: only direct physics-based residual plus smoothing
- `learn`: only PyTorch learning-based estimate
- `both`: run direct estimate and learned estimate together

Typical use:

```bash
--mode both
```

In the current script, learned mode means learning the analytic friction law parameters rather than a binned friction curve.

### `--angle-unit`

Defines the units of the input angle and measured angular velocity columns.

Options:

- `deg`: angle in degrees, omega in deg/s
- `rad`: angle in radians, omega in rad/s

For `SWA.XLS`, use:

```bash
--angle-unit deg
```

### `--time-col`

Column name for time.

Default:

```text
t[s]
```

### `--torque-col`

Column name for TAS torque.

Default:

```text
API_2ms.HwTrq_Nm_s16p10[]
```

### `--angle-col`

Column name for steering angle.

Default:

```text
API_2ms.HwAng_Deg_s32p16[]
```

### `--omega-col`

Column name for measured steering angular velocity.

Default:

```text
API_2ms.HwAngVel_Degs_s32p16[]
```

### `--motor-input-col`

Column name for motor-side input before the Simulink `0.5` gain.

Default:

```text
AimiCurrent[]
```

### `--omega-source`

Chooses which omega signal is used in the dynamics model.

Options:

- `measured`: use the measured omega column directly
- `angle`: derive omega from steering angle using the filtered derivative block

Typical use for `SWA.XLS`:

```bash
--omega-source measured
```

### `--derivative-filter-time`

This is the derivative filter time constant `T`.

Important:

- script currently uses fixed derivative `Ts = 0.1`
- this argument sets `T`, not `Ts`

Examples:

```bash
--derivative-filter-time 0.5
--derivative-filter-time 1.0
```

Interpretation:

- smaller `T`: faster derivative response, noisier `alpha`
- larger `T`: smoother derivative response, slower dynamics

For your current `SWA.XLS`, a practical starting point is:

```bash
--derivative-filter-time 0.5
```

### `--J-nominal`

Nominal inertia used as:

- direct-mode inertia
- initialization center
- regularization center during learning

Default:

```bash
--J-nominal 0.01275
```

Units: `kg*m^2`

### `--B-init`

Initial damping coefficient.

Default:

```bash
--B-init 0.0
```

### `--init-F0`

Initial value for the base friction level `F0` in the analytic law.

Default:

```bash
--init-F0 1.0
```

### `--init-A`

Initial value for the low-speed transition amplitude `A`.

Default:

```bash
--init-A 0.05
```

### `--init-v0`

Initial value for the low-speed exponential transition scale `v0`.

Default:

```bash
--init-v0 0.5
```

### `--init-C1`

Initial value for the linear speed-growth coefficient `C1`.

Default:

```bash
--init-C1 0.01
```

### `--init-omega-eps`

Initial value for the smooth sign transition scale `omega_eps`.

Default:

```bash
--init-omega-eps 0.05
```

### `--use-B`

Controls whether the viscous damping term `B * omega` is included at all.

Options:

- `--use-B`: include damping term
- `--no-use-B`: disable damping term completely

When disabled:

- direct mode uses:

```text
friction = TAS_Torque - 0.5*AimiCurrent - J*alpha
```

- learning mode forces `B = 0`
- `B` is not used in the residual term
- `B` regularization is effectively inactive

Example:

```bash
--no-use-B
```

### `--learn-J`

Enables learning of inertia `J`.

Default: enabled

Examples:

```bash
--learn-J
--no-learn-J
```

### `--learn-B`

Enables learning of damping `B`.

Default: enabled

Examples:

```bash
--learn-B
--no-learn-B
```

### `--epochs`

Number of PyTorch optimization epochs.

Default:

```bash
--epochs 3000
```

Use fewer for quick checks:

```bash
--epochs 100
```

### `--lr`

Learning rate for Adam.

Default:

```bash
--lr 1e-3
```

### `--w-dyn`

Weight for dynamics consistency loss.

This term keeps learned friction close to:

```text
TAS_Torque - 0.5*AimiCurrent - B*omega - J*alpha
```

Default:

```bash
--w-dyn 1.0
```

### `--w-smooth`

Weight for smoothness loss.

This penalizes large changes in `abs(friction_est)` over time.

Default:

```bash
--w-smooth 0.1
```

### `--w-spike`

Weight for spike penalty.

This penalizes unusually large jumps in `abs(friction_est)`.

Default:

```bash
--w-spike 0.1
```

### `--w-sym`

Weight for positive/negative symmetry penalty.

This encourages similar friction magnitudes for positive and negative motion at similar speed.

Default:

```bash
--w-sym 0.1
```

### `--w-J`

Weight for inertia regularization.

This keeps learned `J` near `J_nominal`.

Default:

```bash
--w-J 1.0
```

### `--w-B`

Weight for damping regularization.

This penalizes large learned `B`.

Default:

```bash
--w-B 0.01
```

### `--output`

Path for output CSV.

Default:

```bash
not set explicitly
```

For your current final run, the output file used was:

```bash
--output tuned_final_analytic.csv
```

If omitted, output is automatically placed in the run directory.

### `--plot-dir`

Folder for diagnostic plots.

Default:

```bash
not set explicitly
```

If omitted, plots are automatically placed in:

```text
outputs/runs/<run_name>/plots/
```

### `--run-name`

Optional name for the auto-created run directory.

If omitted, the script builds one automatically from timestamp, data name, mode, and omega source.

Example:

```bash
--run-name auto_layout_check
```

### `--device`

PyTorch device.

Options:

- `cpu`
- `cuda`

Examples:

```bash
--device cpu
--device cuda
```

If `cuda` is selected but unavailable, PyTorch falls back to CPU in the script.

## Notes For `SWA.XLS`

- The file contains metadata rows before the actual header.
- The script auto-detects the real header row.
- The file currently appears to have actual time spacing close to `0.1 s`.
- The derivative block in the script is intentionally run with fixed `Ts = 0.1`.
- `AimiCurrent[]` is not treated as raw electrical current needing another motor constant.
- Friction sign is not forced by `sign(alpha)`.
- The learned model is now an analytic formula in `omega`, not a table over speed bins.
