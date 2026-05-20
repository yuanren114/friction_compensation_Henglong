# Simulink Realtime Adaptation Plan

## Goal

Move the current offline friction-estimation idea into Simulink first, before any ECU deployment.

The first Simulink version should:

- run in real time
- compute a torque residual online
- evaluate the analytic friction law online
- slowly adapt a small number of friction parameters
- avoid full-history storage

The first version should **not** use `B`.

Use:

```text
residual = TAS_Torque - 0.5 * Motor_Input - J * alpha
```

Do not include:

```text
- B * omega
```

## Recommended Top-Level Structure

Create these top-level subsystems:

1. `Signal_Preprocess`
2. `Residual_Calculation`
3. `Analytic_Friction_Model`
4. `Error_Preprocess`
5. `F0_State`
6. `Parameter_Update_Slow`

Suggested signal flow:

```text
TAS_Torque ----\
Motor_Input ----> Residual_Calculation ----> residual ------\
alpha -----------------------------------------------\       \
omega ----------------------------------------------- Error_Preprocess ---> e_slow ---> Parameter_Update_Slow ---> F0_next ---> F0_State ---> F0_live
                                                          \                                              ^                             |
                                                           \--> update_enable -----------------------------/                             |
omega ------------------------------------------------------------------------------------------------------> Analytic_Friction_Model ----/
```

## 1. Signal_Preprocess

### Inputs

- `HwAng` or `TAS angle`
- `HwAngVel` measured

### Outputs

- `theta_rad`
- `omega_measured_rad_s`
- `omega_used`
- `alpha_used`

### First-Version Recommendation

Use measured omega directly.

```text
omega_used = omega_measured_rad_s
alpha_used = filtered_derivative(omega_used)
```

### Simulink Blocks

- `Gain (pi/180)` to convert deg/s into rad/s
- derivative filter block to compute `alpha_used`

## 2. Residual_Calculation

### Inputs

- `TAS_Torque`
- `Motor_Input`
- `alpha_used`
- `J`

### Output

- `residual`

### Formula

```text
motor_term = 0.5 * Motor_Input
inertia_term = J * alpha_used
residual = TAS_Torque - motor_term - inertia_term
```

### Simulink Blocks

- `Gain` with value `0.5`
- `Gain` with value `J`
- `Sum` block with `+--`

### Connection Sketch

```text
Motor_Input --> Gain(0.5) --------\
alpha_used --> Gain(J) ------------> Sum(+--) --> residual
TAS_Torque -----------------------/
```

## 3. Analytic_Friction_Model

### Inputs

- `omega_used`
- `F0`
- `A`
- `v0`
- `C1`
- `omega_eps`

### Output

- `friction_hat`

### Formula

```text
s = tanh(omega / omega_eps)
absw = abs(omega)
phi = 1 - exp(-absw / v0)
mag = F0 + A * phi + C1 * absw
friction_hat = s * mag
```

### Suggested Simulink Decomposition

#### Path A: sign-like smooth term

```text
omega --> Divide by omega_eps --> tanh --> s
```

Use:

- `Divide`
- `Trigonometric Function` if `tanh` is available
- otherwise `MATLAB Function`

#### Path B: absolute speed

```text
omega --> Abs --> absw
```

#### Path C: exponential transition

```text
absw --> Divide by v0 --> Gain(-1) --> exp --> Sum(1 - exp(...)) --> phi
```

#### Path D: magnitude

```text
phi --> Gain(A) -------\
absw --> Gain(C1) ------> Sum(F0 + A*phi + C1*absw) --> mag
F0 --------------------/
```

#### Path E: final friction

```text
s * mag --> friction_hat
```

## 4. Error_Preprocess

This block turns the raw residual mismatch into a more reliable adaptation signal.

### Why This Block Is Needed

Do not use instantaneous `residual - friction_hat` directly for adaptation.

That signal can be corrupted by:

- alpha-estimation noise
- sensor spikes
- direction-change transients
- motor-side mapping mismatch
- unmodeled dynamics

### Inputs

- `residual`
- `friction_hat`
- `omega_used`
- `alpha_used`
- `omega_min`
- `alpha_max`
- `e_beta`
- `e_clip`

### Outputs

- `e_raw`
- `e_slow`
- `update_enable`

### Recommended Logic

```text
e_raw = residual - friction_hat
update_enable = (abs(omega) > omega_min) and (abs(alpha) < alpha_max)
e_used = clip(e_raw, -e_clip, e_clip)

if update_enable:
    e_slow[k] = (1 - beta) * e_slow[k-1] + beta * e_used
else
    e_slow[k] = e_slow[k-1]
```

This makes the adaptation error:

- gated
- clipped
- low-pass filtered

## 5. F0_State

Store the current live value of `F0` as a state.

### Inputs

- `F0_next`
- `F0_init`

### Output

- `F0_live`

### Recommended Simulink Block

- `Unit Delay`

This closes the online adaptation loop:

```text
F0_live -> friction model -> e_slow -> F0_next -> Unit Delay -> F0_live
```

## 6. Parameter_Update_Slow

This is the online adaptation block.

### First-Version Recommendation

Do **not** start by updating all parameters.

First version:

- fix `J`
- fix `v0`
- fix `omega_eps`
- fix `A`
- update only `F0`

Second version:

- update `F0`
- update `C1`

Only later consider updating `A`.

## 4A. Simplest Online Update: Update `F0` Only

### Inputs

- `e_slow`
- `omega_used`
- `update_enable`
- `F0_old`

### Output

- `F0_new`

### Gradient with Respect to `F0`

Because:

```text
friction_hat = s * (F0 + ...)
```

the gradient is:

```text
dF/dF0 = s
```

### Update Rule

```text
F0_new = sat(F0_old + mu_F0 * e_slow * s)
```

Where:

- `mu_F0` is a small update gain
- `sat` is a parameter saturation / clamp
- `e_slow` is the gated and filtered adaptation error

If update is disabled:

```text
F0_new = F0_old
```

## 6B. Next Step: Update `F0` and `C1`

Because:

```text
friction_hat = s * (F0 + A*phi + C1*absw)
```

the gradients are:

```text
dF/dF0 = s
dF/dC1 = s * absw
```

So, using the same `e_slow`:

```text
F0_new = sat(F0_old + mu_F0 * e_slow * s)
C1_new = sat(C1_old + mu_C1 * e_slow * s * absw)
```

## Update_Enable Gating

Do **not** update parameters at every sample without conditions.

### Suggested First-Version Gate

```text
abs(omega) > omega_min
and abs(alpha) < alpha_max
```

### Simulink Implementation

- `Abs`
- `Compare To Constant`
- `Logical Operator (AND)`
- `Switch`

## Parameter Storage

Use:

- `Unit Delay`
or
- `Memory`

to store the previous parameter value.

Example for `F0`:

```text
F0_live --> update law --> Saturation --> Unit Delay --> F0_live(next)
```

## Slow Update Rate

Do not update the parameters at the main fast sample rate.

### Recommended

- fast signal loop: current plant / sensor sample time
- slow parameter update loop: `0.1 s` to `0.5 s`

Two ways:

1. give `Parameter_Update_Slow` a slower sample time
2. keep one sample time and only enable update periodically

For the first Simulink version, the slower-sampled subsystem is simpler.

## Suggested Initial Constants

Use these as a practical starting point:

```text
J = 0.01275
F0 = 0.3
A = 0.0 or 0.02
v0 = 0.5
C1 = 0.0 or 0.01
omega_eps = 0.05
```

If updating only `F0`, keep the others fixed.

## Minimal Deliverable Version

If the requirement is to produce something quickly in Simulink:

### V1

- `omega_used = measured omega`
- `alpha_used = filtered derivative(omega_used)`
- `residual = TAS - 0.5*motor - J*alpha`
- `friction_hat = tanh(...) * (...)`
- `e_slow = gated + clipped + low-pass filtered (residual - friction_hat)`
- online update only `F0` using `e_slow`
- fixed `A`, `v0`, `C1`, `omega_eps`
- add `update_enable`
- add parameter saturation

This already demonstrates:

- real-time execution
- online friction estimation
- slow adaptation
- no need for full-history storage

## Text Diagram

```text
[Measured Omega deg/s] --> [Gain pi/180] --> omega_used -->+--> [Analytic_Friction_Model] --> friction_hat
                                                           |                                      ^
                                                           |                                      |
                                                           +--> [Filtered Derivative] --> alpha --+ 
                                                                                                  |
[TAS_Torque] ---------------------------------------------------------------------------------> [Residual_Calculation] --> residual
[Motor_Input] --> [Gain 0.5] -----------------------------------------------------------------/

residual + friction_hat + omega + alpha --> [Error_Preprocess] --> e_slow --> [Parameter_Update_Slow] --> F0_next --> [F0_State] --> F0_live

F0_live, A, v0, C1, omega_eps --> [Analytic_Friction_Model]
```

## Recommended Next Step

Implement the first Simulink version with:

- no `B`
- fixed `J`
- fixed `A`, `v0`, `C1`, `omega_eps`
- online update of `F0` only
- do not adapt on raw instantaneous residual

Once that behaves well:

- add `C1`
- then consider `A`

Avoid making all parameters adaptive in the first version.

## Replay One XLS in Simulink With Manual `Ts`

If the goal is:

- choose one `XLS`
- feed its logged signals into the Simulink model
- manually choose the derivative-block `Ts`
- observe `F0_live`, `residual`, `friction_hat`, and `alpha_used` in scopes

then use the `From Workspace` method.

### Core Idea

Treat the Excel file as a signal source only.

Do **not** force the derivative block to use the Excel time step automatically.

Instead:

- replay `TAS_Torque`, `Motor_Input`, and `HwAngVel` using the Excel time vector
- feed `Ts` into the Simulink derivative block as a separate constant-valued time series
- feed `Tfilter` the same way

This allows you to test:

- same logged data
- different chosen `Ts`
- effect on `alpha_used`, `residual`, and `F0_live`

### Signals to Feed From Workspace

At minimum, prepare:

- `TAS_Torque_sig`
- `Motor_Input_sig`
- `Angle_sig`
- `Omega_sig`
- `Torque_Hand_sig`
- `Torque_Motor_sig`
- `AimiCurrent_sig`
- `Ts_runtime_sig`
- `Tfilter_runtime_sig`

### Example MATLAB Preparation

For a file such as `FC_constant_spd_test_low.XLS`:

```matlab
file = 'FC_constant_spd_test_low.XLS';
raw = readtable(file, 'FileType', 'spreadsheet', 'ReadVariableNames', true, 'HeaderLines', 6);

t = raw.("t[s]");
angle = raw.("HwAng_Deg_s32p16[]");
omega = raw.("HwAngVel_Degs_s32p16[]");
torque_hand = raw.("HwTrq_Nm_s16p10[]");
torque_motor = 0.5 * raw.("AimiCurrent[]");  % optional replay helper
AimiCurrent = raw.("AimiCurrent[]");

Ts_runtime = 0.002;
Tfilter_runtime = 0.3;

Angle_sig = timeseries(angle, t);
Omega_sig = timeseries(omega, t);
Torque_Hand_sig = timeseries(torque_hand, t);
Torque_Motor_sig = timeseries(torque_motor, t);
AimiCurrent_sig = timeseries(AimiCurrent, t);
Ts_runtime_sig = timeseries(Ts_runtime * ones(size(t)), t);
Tfilter_runtime_sig = timeseries(Tfilter_runtime * ones(size(t)), t);
```

### Simulink Connection Method

At the model top level:

1. replace the external source side with `From Workspace` blocks
2. set their variable names to:
   - `Angle_sig`
   - `Omega_sig`
   - `Torque_Hand_sig`
   - `Torque_Motor_sig`
   - `AimiCurrent_sig`
   - `Ts_runtime_sig`
   - `Tfilter_runtime_sig`
3. connect them to the corresponding places in the model

Recommended mapping for the current online model:

- `Torque_Hand_sig` -> top-level `TAS_Torque`
- `AimiCurrent_sig` -> top-level `Motor_Input`
- `Omega_sig` -> top-level `HwAngVel_Deg_s`
- `Ts_runtime_sig` -> `Signal_Preprocess/Ts`
- `Tfilter_runtime_sig` -> `Signal_Preprocess/Tfilter`

The remaining helper signals are optional monitors / future hooks:

- `Angle_sig` is useful if you later add angle-based logic or want a replay reference
- `Torque_Motor_sig` is useful if you want to monitor the already-halved motor-side contribution separately from `AimiCurrent_sig`

This means:

- the signal values come from the Excel replay
- the derivative block `Ts` is whatever you choose
- the derivative block `Tfilter` is whatever you choose

### What to Observe

Use `Scope` or `Simulation Data Inspector` to watch:

- `F0_live`
- `residual`
- `friction_hat`
- `alpha_used`
- optionally `update_enable`
- optionally `freeze_active`

### Recommended Comparison Experiment

For one chosen file, run the same replay with multiple `Ts` values, for example:

1. `Ts_runtime = median(diff(t))`
2. `Ts_runtime = 0.01`
3. `Ts_runtime = 0.1`

Then compare:

- whether `alpha_used` becomes noisy or unrealistic
- whether `residual` becomes spike-heavy
- whether `F0_live` drifts too aggressively

This is a practical way to study how sensitive the online adaptation is to the derivative-block sampling assumption.
