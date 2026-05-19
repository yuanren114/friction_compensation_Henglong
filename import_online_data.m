file_name = 'FC_constant_spd_test_low_1.XLS';
raw = readtable(file_name, 'FileType', 'spreadsheet', 'ReadVariableNames', true, 'HeaderLines', 6);
raw = rmmissing(raw, 'MinNumMissing', width(raw));

t = raw.("t_s_");
angle = raw.("HwAng_Deg_s32p16__");
omega = raw.("HwAngVel_Degs_s32p16__");
torque_hand = raw.("HwTrq_Nm_s16p10__");
AimiCurrent = raw.("AimiCurrent__");
torque_motor = raw.("ElectricMotorTorque__");


Ts_runtime = 0.01;
Tfilter_runtime = 0.1;

Angle_sig = timeseries(angle, t);
Omega_sig = timeseries(omega, t);
Torque_Hand_sig = timeseries(torque_hand, t);
Torque_Motor_sig = timeseries(torque_motor, t);
AimiCurrent_sig = timeseries(AimiCurrent, t);
Ts_runtime_sig = timeseries(Ts_runtime * ones(size(t)), t);
Tfilter_runtime_sig = timeseries(Tfilter_runtime * ones(size(t)), t);