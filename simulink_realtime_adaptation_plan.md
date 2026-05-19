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
4. `Parameter_Update_Slow`

Suggested signal flow:

```text
TAS_Torque ----\
Motor_Input ----> Residual_Calculation ----> residual ----\
omega ---------/                                      |    \
alpha -----------------------------------------------/      \
                                                             > compare / update
omega ------------------------------------------------------> Analytic_Friction_Model --> friction_hat
                                                                                   ^
                                                                                   |
                                                               Parameter_Update_Slow|
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

## 4. Parameter_Update_Slow

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

- `residual`
- `friction_hat`
- `omega_used`
- `update_enable`
- `F0_old`

### Output

- `F0_new`

### Error

```text
e = residual - friction_hat
```

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
F0_new = sat(F0_old + mu_F0 * e * s)
```

Where:

- `mu_F0` is a small update gain
- `sat` is a parameter saturation / clamp

If update is disabled:

```text
F0_new = F0_old
```

## 4B. Next Step: Update `F0` and `C1`

Because:

```text
friction_hat = s * (F0 + A*phi + C1*absw)
```

the gradients are:

```text
dF/dF0 = s
dF/dC1 = s * absw
```

So:

```text
F0_new = sat(F0_old + mu_F0 * e * s)
C1_new = sat(C1_old + mu_C1 * e * s * absw)
```

## Update_Enable Gating

Do **not** update parameters at every sample without conditions.

### Suggested First-Version Gate

```text
abs(omega) > omega_min
and abs(alpha) < alpha_max
and abs(residual) < residual_max
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
F0_old --> update law --> Saturation --> Unit Delay --> F0_old(next)
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
- online update only `F0`
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
                                                           |
                                                           +--> [Filtered Derivative] --> alpha_used --> [Gain J] --\
                                                                                                                      \
[TAS_Torque] -----------------------------------------------------------------------------------------------> [Sum] --> residual
[Motor_Input] --> [Gain 0.5] -------------------------------------------------------------------------------/

residual + friction_hat + omega_used --> [Parameter_Update_Slow] --> F0 (and later C1)

F0, A, v0, C1, omega_eps --> [Analytic_Friction_Model]
```

## Recommended Next Step

Implement the first Simulink version with:

- no `B`
- fixed `J`
- fixed `A`, `v0`, `C1`, `omega_eps`
- online update of `F0` only

Once that behaves well:

- add `C1`
- then consider `A`

Avoid making all parameters adaptive in the first version.
