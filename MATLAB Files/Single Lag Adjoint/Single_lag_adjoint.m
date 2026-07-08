%% Set up
clc
clear
close all


%% Parameters & Variables
p.G = 9.81;
p.navigation_ratio = 4;
p.missile_speed_mps = 3000;
p.target_acceleration_mps2 = 8 * p.G;
p.heading_error_rad = deg2rad(-5);
p.time_constant = 0.2;
p.T_final = 10;
p.H = 0.0001;
p.tGo_initial_s = 1e-5;
p.tGo_min_s = 1e-5;

x = [0,0,1,0];

tGo_s = p.tGo_initial_s;
log_step = 100 * p.H;
log_stride = round(log_step/p.H);
num_logs = floor((p.T_final - p.tGo_initial_s)/log_step) + 1;
num_steps = floor(p.T_final / p.H + 1);
log_idx = 1;

log.time_s = zeros(num_logs,1);
log.miss_maneuver_m = zeros(num_logs,1);
log.miss_heading_m = zeros(num_logs,1);

%% Runge-Katta
for step_idx = 1:num_steps

    if mod(step_idx-1, log_stride) == 0
        log.time_s(log_idx) = tGo_s;
        log.miss_maneuver_m(log_idx) = p.target_acceleration_mps2 * x(1);
        log.miss_heading_m(log_idx) = -p.missile_speed_mps * p.heading_error_rad * x(2);
        log_idx = log_idx+1;  
    end

    if step_idx == num_steps
        break
    end


    k1 = adjoint_rhs(tGo_s,          x, p);
    k2 = adjoint_rhs(tGo_s + p.H/2,  x + k1 * p.H/2,     p);
    k3 = adjoint_rhs(tGo_s + p.H/2,  x + k2 * p.H/2,     p);
    k4 = adjoint_rhs(tGo_s + p.H,    x + k3 * p.H,       p);

    x = x + p.H/6 * (k1 + 2*k2 + 2*k3 + k4);

    tGo_s = tGo_s + p.H;
end


%% Results
results = table( ...
    log.time_s, ...
    log.miss_maneuver_m, ...
    log.miss_heading_m, ...
    'VariableNames', {'time_s', 'miss_maneuver_m', 'miss_heading_m'} ...
    );

fig = figure;
tiledlayout(2,1)

nexttile
plot(results.time_s, results.miss_maneuver_m)
grid on
xlabel('Time to Go (s)')
ylabel('Miss due to Target Maneuver (m)')

nexttile
plot(results.time_s, results.miss_heading_m)
grid on
xlabel('Time to Go (s)')
ylabel('Miss due to Heading Error (m)')

parameter_text = sprintf([ ...
    'N = %.2f\n' ...
    'V_M = %.1f m/s\n' ...
    'n_T = %.2f m/s^2\n' ...
    'HE = %.1f deg\n' ...
    'tau = %.2f s\n' ...
    'H = %.3f s'], ...
    p.navigation_ratio, ...
    p.missile_speed_mps, ...
    p.target_acceleration_mps2, ...
    rad2deg(p.heading_error_rad), ...
    p.time_constant, ...
    p.H);

annotation('textbox', [0.68 0.72 0.22 0.18], ...
    'String', parameter_text, ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', 'white', ...
    'EdgeColor', 'black');

save("Single Lag Adjoint Results.mat", "results", "p");
savefig(fig, "Single Lag Adjoint Figure.fig")
writetable(results,"Single Lag Adjoint Results.csv");

%% Functions
function xd = adjoint_rhs(t, x, p)
    t_go_s = max(t, p.tGo_min_s);
    
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);

    y1 = (x4-x2)/p.time_constant;
    x1d = x2;
    x2d = x3;
    x3d = (y1*p.navigation_ratio)/t_go_s;
    x4d = -y1;
    
    xd = [x1d, x2d, x3d, x4d];
end
