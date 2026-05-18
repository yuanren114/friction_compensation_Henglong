# friction_estimator design

## Goal

`friction_estimator.py` does not assume the raw torque residual is already the final friction law.

It separates the problem into two layers:

1. a dynamics residual model
2. an analytic friction law fit to that residual

The goal is to get a friction estimate that is:

- physically interpretable
- smoother than the raw residual
- still tied to measured dynamics
- usable on future data through an explicit formula

## Signal Flow

For a given run:

1. load torque, angle, measured omega, and motor-side input
2. convert angle and omega into radians / rad/s
3. choose `omega_used`
4. compute `alpha_used` using the filtered derivative block
5. compute a dynamics residual
6. fit an analytic friction law to that residual

The main outputs are:

- `friction_raw`
- `friction_direct_smoothed`
- `friction_learned`
- `dynamics_residual_learned`

## Dynamics Model

The script uses the residual structure:

```text
residual[k] = T_TAS[k] - G_motor * U_motor[k] - B * omega[k] - J * alpha[k]
```

In this project:

```text
G_motor = 0.5
```

For the current dataset, `U_motor` is `AimiCurrent[]`, interpreted as the Simulink-side input before the `0.5` gain.

If `--no-use-B` is used, then:

```text
residual[k] = T_TAS[k] - 0.5 * U_motor[k] - J * alpha[k]
```

## Why `alpha` Uses a Filtered Derivative

`alpha` is the most noise-sensitive term in the model because it is computed from a derivative of angular velocity.

Direct finite differences would amplify:

- quantization
- sparse sampling
- jitter
- small measurement noise

So the script uses a filtered derivative:

```text
X[k] = (Ts / T) * u[k] + (1 - Ts / T) * X[k-1]
alpha[k] = (1 / T) * (u[k] - X[k-1])
```

Where:

- `u[k]` is the chosen omega signal
- `Ts` is the fixed derivative update step used by the script
- `T` is `--derivative-filter-time`

This gives a more stable acceleration estimate than a raw difference.

## Friction Law

The learned friction model is analytic:

```text
F_fric(omega) = tanh(omega / omega_eps) * (F0 + A * (1 - exp(-|omega| / v0)) + C1 * |omega|)
```

### Meaning of Each Term

`tanh(omega / omega_eps)`

- provides a smooth sign transition
- avoids a hard discontinuity at zero speed
- keeps friction sign aligned with motion direction

`F0`

- base friction level
- dominant low-speed / baseline amplitude

`A * (1 - exp(-|omega| / v0))`

- low-speed rise / transition term
- allows friction magnitude to move away from the base level near low speed
- saturates as `|omega|` increases

`C1 * |omega|`

- allows a linear increase with speed magnitude
- keeps the magnitude model symmetric because it uses `|omega|`

This model is odd overall because the sign comes from `tanh(omega / omega_eps)` and the magnitude depends only on `|omega|`.

That means:

```text
F_fric(-omega) = -F_fric(omega)
```

## Why the Parameters Use `raw_*` + `softplus`

The model stores trainable parameters in raw form and transforms them with `softplus`.

For example:

```python
F0 = softplus(raw_F0)
```

This ensures:

- `F0 >= 0`
- `A >= 0`
- `v0 > 0`
- `C1 >= 0`
- `omega_eps > 0`

This makes the fitted formula easier to interpret and avoids unphysical negative magnitudes or invalid time scales.

## Learning Target

The analytic friction law is not trained against `friction_direct_smoothed`.

It is trained against the dynamics residual:

```text
friction_learned[k] ≈ dynamics_residual_learned[k]
```

Where:

```text
dynamics_residual_learned[k] = T_TAS[k] - G_motor * U_motor[k] - B_learned * omega[k] - J_learned * alpha[k]
```

So:

- `dynamics_residual_learned` is the model-based residual target
- `friction_learned` is the analytic law trying to explain that target

## Loss Design

The total loss is:

```text
loss =
    w_dyn * L_dyn
  + w_smooth * L_smooth
  + w_spike * L_spike
  + w_sym * L_sym
  + w_J * L_J
  + w_B * L_B
```

### 1. Dynamics Consistency

```text
L_dyn = Huber(friction_est, residual)
```

Purpose:

- keep the analytic friction law close to the dynamics residual
- reduce sensitivity to large outliers compared with plain MSE

Why Huber:

- quadratic near zero
- linear for larger errors
- more robust when residual spikes exist

### 2. Smoothness

Let:

```text
abs_f[k] = |friction_est[k]|
abs_diff[k] = abs_f[k+1] - abs_f[k]
```

Then:

```text
L_smooth = mean(abs_diff^2)
```

Purpose:

- discourage rapid sample-to-sample changes in friction magnitude
- keep the learned friction law from chasing small residual fluctuations

### 3. Spike Penalty

Define:

```text
jump[k] = |abs_diff[k]|
threshold = quantile(jump, 0.95)
L_spike = mean(relu(jump - threshold)^2)
```

Purpose:

- identify unusually large local jumps
- penalize only the top end of jump sizes

Why the quantile:

- the threshold adapts to the scale of the current solution
- no fixed absolute spike threshold is required

### 4. Symmetry Penalty

The data is binned by `|omega|`.

Within each bin:

- compute median `|friction_est|` for `omega > 0`
- compute median `|friction_est|` for `omega < 0`

Then penalize their difference:

```text
L_sym = mean((median_pos - median_neg)^2)
```

Purpose:

- encourage positive and negative motion to have similar friction magnitude
- keep the learned model consistent with an odd friction law

### 5. Inertia Regularization

```text
L_J = ((J - J_nominal) / J_nominal)^2
```

Purpose:

- allow `J` to adapt
- prevent it from drifting too far from the nominal physical estimate

### 6. Damping Regularization

```text
L_B = B^2
```

Purpose:

- discourage large damping values
- stop `B` from absorbing too much of what should be modeled as friction

If `--no-use-B` is selected, the damping term is disabled and `L_B = 0`.

## Optimization Principle

The optimizer is Adam.

At each epoch:

1. evaluate the analytic friction law on the full sequence
2. compute the residual with current `J` and `B`
3. compute all loss terms
4. backpropagate through the analytic parameters and any enabled dynamic parameters
5. update parameters

In the current implementation, the optimization is effectively full-batch:

- one epoch uses the whole sequence
- one epoch is approximately one parameter-update iteration

## Why Total Loss Can Change Only a Little

In many runs:

- `L_dyn` is the dominant term
- the regularization terms are much smaller

So total loss may move only slightly, even when the analytic parameters change noticeably.

This usually means one or more of:

- the initial analytic law was already reasonable
- the current data only weakly constrains the parameters
- several parameter combinations produce similar residual fit

That is why:

- parameter values may move
- friction shape may change
- total loss may still look almost flat

## Interpreting Key Outputs

### `friction_raw`

Direct residual using the current direct-mode `J` and `B`.

### `friction_direct_smoothed`

Signal-processing-smoothed version of `friction_raw`.

Useful as a reference, but not the target for learning.

### `friction_learned`

Analytic friction law output.

This is the main learned friction estimate.

### `dynamics_residual_learned`

Residual recomputed using the final learned `J` and `B`.

This is the quantity `friction_learned` is trying to explain.

### `loss_history`

Per-epoch tracking of:

- `loss`
- `L_dyn`
- `L_smooth`
- `L_spike`
- `L_sym`
- `L_J`
- `L_B`
- `J`
- `J_scale`
- `B`

Useful for checking:

- monotonic convergence
- whether smoothness/spike penalties are rising or falling
- whether the model is trading `L_dyn` against regularization

## Practical Interpretation

When comparing runs:

- lower `L_dyn` means better agreement with the residual
- lower `L_smooth` and `L_spike` means a calmer analytic friction law
- lower `L_sym` means better positive/negative consistency
- a good run is usually a tradeoff, not the absolute minimum of every term

For this reason, model selection should look at both:

- the summary table / loss values
- the plots of friction vs time and friction vs `|omega|`
