function build_friction_rt_model(model_name)
%BUILD_FRICTION_RT_MODEL Create a first-pass Simulink model for online friction adaptation.
%
% This script programmatically builds a Simulink model that mirrors the
% current reduced onboard-oriented structure:
%
%   residual = TAS_Torque - 0.5 * Motor_Input - J * alpha
%
%   friction_hat = tanh(omega / omega_eps) * ...
%       (F0 + A * (1 - exp(-abs(omega) / v0)) + C1 * abs(omega))
%
% Design assumptions for the first version:
% - B is intentionally omitted.
% - measured steering angular velocity is used directly as omega.
% - alpha is obtained from a filtered derivative block.
% - J, A, v0, C1, omega_eps are constants in the initial model.
% - F0 is updated slowly online by a MATLAB Function block.
% - the adaptation does not use raw instantaneous residual directly; it uses
%   a gated low-pass filtered error signal.
% - adaptation freeze is entered by anomaly trigger and released by recovery
%   conditions instead of a fixed freeze window.
%
% Usage:
%   build_friction_rt_model
%   build_friction_rt_model('friction_rt_online')

if nargin < 1 || isempty(model_name)
    model_name = 'friction_rt_online';
end

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

if exist([model_name '.slx'], 'file')
    delete([model_name '.slx']);
end

new_system(model_name);
open_system(model_name);
set_param(model_name, 'StopTime', '10');

layout = get_layout();
add_top_level_ports(model_name, layout);
add_parameter_constants(model_name, layout);
add_signal_preprocess(model_name, layout);
add_residual_calculation(model_name, layout);
add_analytic_friction_model(model_name, layout);
add_error_preprocess(model_name, layout);
add_parameter_update(model_name, layout);
add_f0_state(model_name, layout);
wire_top_level(model_name);

Simulink.BlockDiagram.arrangeSystem(model_name);
save_system(model_name);
fprintf('Created Simulink model: %s.slx\n', model_name);
end

function layout = get_layout()
layout.in_x = 40;
layout.const_x = 40;
layout.mid_x = 330;
layout.sub_x = 270;
layout.out_x = 980;

layout.tas_y = 60;
layout.motor_y = 120;
layout.omega_y = 180;
layout.temp_y = 240;

layout.const0_y = 340;
layout.const_dy = 50;

layout.pre_y = 40;
layout.residual_y = 220;
layout.friction_y = 420;
layout.err_y = 620;
layout.state_y = 760;
layout.update_y = 900;

layout.scope_x = 1180;
layout.scope_y = 250;
end

function add_top_level_ports(model_name, layout)
add_block('simulink/Sources/In1', [model_name '/TAS_Torque'], ...
    'Position', [layout.in_x layout.tas_y layout.in_x+30 layout.tas_y+20]);
add_block('simulink/Sources/In1', [model_name '/Motor_Input'], ...
    'Position', [layout.in_x layout.motor_y layout.in_x+30 layout.motor_y+20]);
add_block('simulink/Sources/In1', [model_name '/HwAngVel_Deg_s'], ...
    'Position', [layout.in_x layout.omega_y layout.in_x+30 layout.omega_y+20]);
add_block('simulink/Sources/In1', [model_name '/MCU_Temperature'], ...
    'Position', [layout.in_x layout.temp_y layout.in_x+30 layout.temp_y+20]);

add_block('simulink/Sinks/Out1', [model_name '/friction_hat'], ...
    'Position', [layout.out_x 460 layout.out_x+30 480]);
add_block('simulink/Sinks/Out1', [model_name '/residual'], ...
    'Position', [layout.out_x 280 layout.out_x+30 300]);
add_block('simulink/Sinks/Out1', [model_name '/alpha_used'], ...
    'Position', [layout.out_x 120 layout.out_x+30 140]);
add_block('simulink/Sinks/Out1', [model_name '/F0_live'], ...
    'Position', [layout.out_x 700 layout.out_x+30 720]);
add_block('simulink/Sinks/Out1', [model_name '/e_slow'], ...
    'Position', [layout.out_x 780 layout.out_x+30 800]);
add_block('simulink/Sinks/Out1', [model_name '/update_enable'], ...
    'Position', [layout.out_x 840 layout.out_x+30 860]);
add_block('simulink/Sinks/Out1', [model_name '/freeze_active'], ...
    'Position', [layout.out_x 900 layout.out_x+30 920]);
end

function add_parameter_constants(model_name, layout)
names = {'J_const','A_const','v0_const','C1_const','omega_eps_const', ...
         'Ts_const','Tfilter_const','F0_init_const','mu_F0_const', ...
         'F0_min_const','F0_max_const','omega_min_const','alpha_max_const', ...
         'e_beta_const','e_clip_const','e_high_const','e_low_const','recover_needed_const'};
values = {'0.01275','0.02','0.5','0.01','0.05', ...
          '0.1','0.5','0.3','1e-4', ...
          '0.05','2.0','0.05','20.0', ...
          '0.05','1.0','0.6','0.2','5'};

for i = 1:numel(names)
    y = layout.const0_y + (i-1) * layout.const_dy;
    add_block('simulink/Sources/Constant', [model_name '/' names{i}], ...
        'Value', values{i}, ...
        'Position', [layout.const_x y layout.const_x+80 y+25]);
end
end

function add_signal_preprocess(model_name, layout)
sub = [model_name '/Signal_Preprocess'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [layout.sub_x layout.pre_y layout.sub_x+220 layout.pre_y+130]);

open_system(sub);
delete_default_contents(sub);

add_block('simulink/Sources/In1', [sub '/HwAngVel_Deg_s'], ...
    'Position', [30 38 60 58]);
add_block('simulink/Sources/In1', [sub '/Ts'], ...
    'Position', [30 88 60 108]);
add_block('simulink/Sources/In1', [sub '/Tfilter'], ...
    'Position', [30 138 60 158]);

add_block('simulink/Math Operations/Gain', [sub '/DegToRad'], ...
    'Gain', 'pi/180', ...
    'Position', [100 35 155 61]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/FilteredDerivative'], ...
    'Position', [205 25 365 170]);
set_matlab_function_script([sub '/FilteredDerivative'], filtered_derivative_code());

add_block('simulink/Sinks/Out1', [sub '/omega_used'], ...
    'Position', [420 48 450 68]);
add_block('simulink/Sinks/Out1', [sub '/alpha_used'], ...
    'Position', [420 108 450 128]);

add_line(sub, 'HwAngVel_Deg_s/1', 'DegToRad/1');
add_line(sub, 'DegToRad/1', 'FilteredDerivative/1');
add_line(sub, 'Ts/1', 'FilteredDerivative/2');
add_line(sub, 'Tfilter/1', 'FilteredDerivative/3');
add_line(sub, 'DegToRad/1', 'omega_used/1');
add_line(sub, 'FilteredDerivative/1', 'alpha_used/1');
end

function add_residual_calculation(model_name, layout)
sub = [model_name '/Residual_Calculation'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [layout.sub_x layout.residual_y layout.sub_x+260 layout.residual_y+120]);

open_system(sub);
delete_default_contents(sub);

add_block('simulink/Sources/In1', [sub '/TAS_Torque'], ...
    'Position', [30 28 60 48]);
add_block('simulink/Sources/In1', [sub '/Motor_Input'], ...
    'Position', [30 78 60 98]);
add_block('simulink/Sources/In1', [sub '/alpha_used'], ...
    'Position', [30 128 60 148]);
add_block('simulink/Sources/In1', [sub '/J'], ...
    'Position', [30 178 60 198]);

add_block('simulink/Math Operations/Gain', [sub '/MotorHalfGain'], ...
    'Gain', '0.5', ...
    'Position', [100 75 160 101]);
add_block('simulink/Math Operations/Product', [sub '/InertiaTerm'], ...
    'Position', [105 165 165 205]);
add_block('simulink/Math Operations/Sum', [sub '/ResidualSum'], ...
    'Inputs', '+--', ...
    'Position', [215 72 245 148]);
add_block('simulink/Sinks/Out1', [sub '/residual'], ...
    'Position', [300 98 330 118]);

add_line(sub, 'TAS_Torque/1', 'ResidualSum/1');
add_line(sub, 'Motor_Input/1', 'MotorHalfGain/1');
add_line(sub, 'MotorHalfGain/1', 'ResidualSum/2');
add_line(sub, 'alpha_used/1', 'InertiaTerm/1');
add_line(sub, 'J/1', 'InertiaTerm/2');
add_line(sub, 'InertiaTerm/1', 'ResidualSum/3');
add_line(sub, 'ResidualSum/1', 'residual/1');
end

function add_analytic_friction_model(model_name, layout)
sub = [model_name '/Analytic_Friction_Model'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [layout.sub_x layout.friction_y layout.sub_x+360 layout.friction_y+210]);

open_system(sub);
delete_default_contents(sub);

inputs = {'omega_used','F0','A','v0','C1','omega_eps'};
for i = 1:numel(inputs)
    y = 28 + (i-1) * 45;
    add_block('simulink/Sources/In1', [sub '/' inputs{i}], ...
        'Position', [30 y 60 y+20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/FrictionLaw'], ...
    'Position', [120 60 330 230]);
set_matlab_function_script([sub '/FrictionLaw'], friction_law_code());

add_block('simulink/Sinks/Out1', [sub '/friction_hat'], ...
    'Position', [395 133 425 153]);

for i = 1:numel(inputs)
    add_line(sub, [inputs{i} '/1'], ['FrictionLaw/' num2str(i)]);
end
add_line(sub, 'FrictionLaw/1', 'friction_hat/1');
end

function add_error_preprocess(model_name, layout)
sub = [model_name '/Error_Preprocess'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [layout.sub_x layout.err_y layout.sub_x+360 layout.err_y+190]);

open_system(sub);
delete_default_contents(sub);

inputs = {'residual','friction_hat','omega_used','alpha_used','omega_min','alpha_max', ...
          'e_beta','e_clip','e_high','e_low','recover_needed'};
for i = 1:numel(inputs)
    y = 28 + (i-1) * 30;
    add_block('simulink/Sources/In1', [sub '/' inputs{i}], ...
        'Position', [30 y 60 y+20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/FilterError'], ...
    'Position', [120 45 310 250]);
set_matlab_function_script([sub '/FilterError'], filter_error_code());

add_block('simulink/Sinks/Out1', [sub '/e_raw'], ...
    'Position', [360 78 390 98]);
add_block('simulink/Sinks/Out1', [sub '/e_slow'], ...
    'Position', [360 138 390 158]);
add_block('simulink/Sinks/Out1', [sub '/update_enable'], ...
    'Position', [360 198 390 218]);
add_block('simulink/Sinks/Out1', [sub '/freeze_active'], ...
    'Position', [360 258 390 278]);

for i = 1:numel(inputs)
    add_line(sub, [inputs{i} '/1'], ['FilterError/' num2str(i)]);
end
add_line(sub, 'FilterError/1', 'e_raw/1');
add_line(sub, 'FilterError/2', 'e_slow/1');
add_line(sub, 'FilterError/3', 'update_enable/1');
add_line(sub, 'FilterError/4', 'freeze_active/1');
end

function add_parameter_update(model_name, layout)
sub = [model_name '/Parameter_Update_Slow'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [layout.sub_x layout.update_y layout.sub_x+420 layout.update_y+180]);
set_param(sub, 'TreatAsAtomicUnit', 'on');

open_system(sub);
delete_default_contents(sub);

inputs = {'e_slow','omega_used','update_enable','omega_eps', ...
          'F0_prev','mu_F0','F0_min','F0_max'};
for i = 1:numel(inputs)
    y = 28 + (i-1) * 36;
    add_block('simulink/Sources/In1', [sub '/' inputs{i}], ...
        'Position', [30 y 60 y+20]);
end

add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/UpdateF0'], ...
    'Position', [120 55 335 240]);
set_matlab_function_script([sub '/UpdateF0'], update_f0_code());

add_block('simulink/Sinks/Out1', [sub '/F0_next'], ...
    'Position', [390 128 420 148]);

for i = 1:numel(inputs)
    add_line(sub, [inputs{i} '/1'], ['UpdateF0/' num2str(i)]);
end
add_line(sub, 'UpdateF0/1', 'F0_next/1');
end

function add_f0_state(model_name, layout)
sub = [model_name '/F0_State'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [layout.sub_x layout.state_y layout.sub_x+280 layout.state_y+120]);
set_param(sub, 'TreatAsAtomicUnit', 'on');

open_system(sub);
delete_default_contents(sub);

add_block('simulink/Sources/In1', [sub '/F0_next'], ...
    'Position', [30 38 60 58]);
add_block('simulink/Sources/In1', [sub '/F0_init'], ...
    'Position', [30 88 60 108]);
add_block('simulink/Discrete/Delay', [sub '/F0_delay'], ...
    'Position', [125 30 180 80], ...
    'DelayLength', '1', ...
    'InitialConditionSource', 'Input port');
add_block('simulink/Sinks/Out1', [sub '/F0_live'], ...
    'Position', [230 43 260 63]);

add_line(sub, 'F0_next/1', 'F0_delay/1');
add_line(sub, 'F0_init/1', 'F0_delay/2');
add_line(sub, 'F0_delay/1', 'F0_live/1');
end

function wire_top_level(model_name)
add_line(model_name, 'HwAngVel_Deg_s/1', 'Signal_Preprocess/1');
add_line(model_name, 'Ts_const/1', 'Signal_Preprocess/2');
add_line(model_name, 'Tfilter_const/1', 'Signal_Preprocess/3');

add_line(model_name, 'Signal_Preprocess/1', 'Analytic_Friction_Model/1');
add_line(model_name, 'Signal_Preprocess/2', 'Residual_Calculation/3');

add_line(model_name, 'TAS_Torque/1', 'Residual_Calculation/1');
add_line(model_name, 'Motor_Input/1', 'Residual_Calculation/2');
add_line(model_name, 'J_const/1', 'Residual_Calculation/4');

add_line(model_name, 'Residual_Calculation/1', 'Error_Preprocess/1');
add_line(model_name, 'Analytic_Friction_Model/1', 'Error_Preprocess/2');
add_line(model_name, 'Signal_Preprocess/1', 'Error_Preprocess/3');
add_line(model_name, 'Signal_Preprocess/2', 'Error_Preprocess/4');
add_line(model_name, 'omega_min_const/1', 'Error_Preprocess/5');
add_line(model_name, 'alpha_max_const/1', 'Error_Preprocess/6');
add_line(model_name, 'e_beta_const/1', 'Error_Preprocess/7');
add_line(model_name, 'e_clip_const/1', 'Error_Preprocess/8');
add_line(model_name, 'e_high_const/1', 'Error_Preprocess/9');
add_line(model_name, 'e_low_const/1', 'Error_Preprocess/10');
add_line(model_name, 'recover_needed_const/1', 'Error_Preprocess/11');

add_line(model_name, 'Error_Preprocess/2', 'Parameter_Update_Slow/1');
add_line(model_name, 'Signal_Preprocess/1', 'Parameter_Update_Slow/2');
add_line(model_name, 'Error_Preprocess/3', 'Parameter_Update_Slow/3');
add_line(model_name, 'omega_eps_const/1', 'Parameter_Update_Slow/4');
add_line(model_name, 'F0_State/1', 'Parameter_Update_Slow/5');
add_line(model_name, 'mu_F0_const/1', 'Parameter_Update_Slow/6');
add_line(model_name, 'F0_min_const/1', 'Parameter_Update_Slow/7');
add_line(model_name, 'F0_max_const/1', 'Parameter_Update_Slow/8');

add_line(model_name, 'Parameter_Update_Slow/1', 'F0_State/1');
add_line(model_name, 'F0_init_const/1', 'F0_State/2');

add_line(model_name, 'F0_State/1', 'Analytic_Friction_Model/2');
add_line(model_name, 'A_const/1', 'Analytic_Friction_Model/3');
add_line(model_name, 'v0_const/1', 'Analytic_Friction_Model/4');
add_line(model_name, 'C1_const/1', 'Analytic_Friction_Model/5');
add_line(model_name, 'omega_eps_const/1', 'Analytic_Friction_Model/6');

add_line(model_name, 'Analytic_Friction_Model/1', 'friction_hat/1');
add_line(model_name, 'Residual_Calculation/1', 'residual/1');
add_line(model_name, 'Signal_Preprocess/2', 'alpha_used/1');
add_line(model_name, 'F0_State/1', 'F0_live/1');
add_line(model_name, 'Error_Preprocess/2', 'e_slow/1');
add_line(model_name, 'Error_Preprocess/3', 'update_enable/1');
add_line(model_name, 'Error_Preprocess/4', 'freeze_active/1');

add_block('simulink/Sinks/Scope', [model_name '/DiagnosticsScope'], ...
    'Position', [1180 250 1210 330], ...
    'NumInputPorts', '6');
add_line(model_name, 'Signal_Preprocess/1', 'DiagnosticsScope/1');
add_line(model_name, 'Signal_Preprocess/2', 'DiagnosticsScope/2');
add_line(model_name, 'Residual_Calculation/1', 'DiagnosticsScope/3');
add_line(model_name, 'Analytic_Friction_Model/1', 'DiagnosticsScope/4');
add_line(model_name, 'Error_Preprocess/2', 'DiagnosticsScope/5');
add_line(model_name, 'Error_Preprocess/4', 'DiagnosticsScope/6');
end

function delete_default_contents(sub)
blocks = find_system(sub, 'SearchDepth', 1, 'Type', 'Block');
for i = 2:numel(blocks)
    delete_block(blocks{i});
end
lines = find_system(sub, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'Line');
for i = 1:numel(lines)
    delete_line(lines(i));
end
end

function set_matlab_function_script(block_path, code)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
if isempty(chart)
    error('Could not find MATLAB Function chart for block: %s', block_path);
end
chart.Script = char(code);
end

function code = filtered_derivative_code()
code = [ ...
"function alpha = fcn(omega, Ts, Tfilter)" newline ...
"% Filtered derivative used for the inertia term." newline ...
"% X[k] = (Ts/T) * omega[k] + (1 - Ts/T) * X[k-1]" newline ...
"% alpha[k] = (1/T) * (omega[k] - X[k-1])" newline ...
"persistent x_prev" newline ...
"if isempty(x_prev)" newline ...
"    x_prev = omega;" newline ...
"end" newline ...
"a = Ts / max(Tfilter, 1e-6);" newline ...
"a = min(max(a, 0.0), 1.0);" newline ...
"alpha = (omega - x_prev) / max(Tfilter, 1e-6);" newline ...
"x_new = a * omega + (1.0 - a) * x_prev;" newline ...
"x_prev = x_new;" newline];
end

function code = friction_law_code()
code = [ ...
"function friction_hat = fcn(omega, F0, A, v0, C1, omega_eps)" newline ...
"% Analytic friction law:" newline ...
"% friction_hat = tanh(omega/omega_eps) * (F0 + A*(1-exp(-abs(omega)/v0)) + C1*abs(omega))" newline ...
"absw = abs(omega);" newline ...
"s = tanh(omega / max(omega_eps, 1e-6));" newline ...
"phi = 1.0 - exp(-absw / max(v0, 1e-6));" newline ...
"mag = F0 + A * phi + C1 * absw;" newline ...
"if mag < 0.0" newline ...
"    mag = 0.0;" newline ...
"end" newline ...
"friction_hat = s * mag;" newline];
end

function code = update_f0_code()
code = [ ...
"function F0_next = fcn(e_slow, omega, update_enable, omega_eps, F0_prev, mu_F0, F0_min, F0_max)" newline ...
"% Slow F0 adaptation using a filtered error instead of raw instantaneous residual." newline ...
"s = tanh(omega / max(omega_eps, 1e-6));" newline ...
"F0_next = F0_prev;" newline ...
"if update_enable" newline ...
"    F0_next = F0_prev + mu_F0 * e_slow * s;" newline ...
"end" newline ...
"if F0_next < F0_min" newline ...
"    F0_next = F0_min;" newline ...
"elseif F0_next > F0_max" newline ...
"    F0_next = F0_max;" newline ...
"end" newline];
end

function code = filter_error_code()
code = [ ...
"function [e_raw, e_slow, update_enable, freeze_active] = fcn(residual, friction_hat, omega, alpha, omega_min, alpha_max, e_beta, e_clip, e_high, e_low, recover_needed)" newline ...
"% Build a reliable adaptation error." newline ...
"% Enter freeze on anomaly; exit freeze only after recovery is sustained." newline ...
"persistent e_prev freeze_state recover_count" newline ...
"if isempty(e_prev)" newline ...
"    e_prev = 0.0;" newline ...
"    freeze_state = false;" newline ...
"    recover_count = 0;" newline ...
"end" newline ...
"e_raw = residual - friction_hat;" newline ...
"trusted_sample = (abs(omega) > omega_min) && (abs(alpha) < alpha_max);" newline ...
"anomaly = (abs(e_raw) > e_high) || (abs(alpha) > alpha_max);" newline ...
"recovered = trusted_sample && (abs(e_raw) < e_low);" newline ...
"if ~freeze_state && anomaly" newline ...
"    freeze_state = true;" newline ...
"    recover_count = 0;" newline ...
"elseif freeze_state" newline ...
"    if recovered" newline ...
"        recover_count = recover_count + 1;" newline ...
"        if recover_count >= recover_needed" newline ...
"            freeze_state = false;" newline ...
"            recover_count = 0;" newline ...
"        end" newline ...
"    else" newline ...
"        recover_count = 0;" newline ...
"    end" newline ...
"end" newline ...
"freeze_active = freeze_state;" newline ...
"update_enable = trusted_sample && ~freeze_state;" newline ...
"e_used = e_raw;" newline ...
"if e_used > e_clip" newline ...
"    e_used = e_clip;" newline ...
"elseif e_used < -e_clip" newline ...
"    e_used = -e_clip;" newline ...
"end" newline ...
"if update_enable" newline ...
"    beta = min(max(e_beta, 0.0), 1.0);" newline ...
"    e_prev = (1.0 - beta) * e_prev + beta * e_used;" newline ...
"end" newline ...
"e_slow = e_prev;" newline];
end
