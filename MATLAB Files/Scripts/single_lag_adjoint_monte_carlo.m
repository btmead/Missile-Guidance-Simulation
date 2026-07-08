clc
clear
close all

gpuDevice;
%rng(1); %Seed (for debugging)
%% Parameters & Variables
p.G = 9.81;
p.navigation_ratio = 4;
p.missile_speed_mps = 1715;
p.target_acceleration_mps2 = 8 * p.G;
p.heading_error_rad = deg2rad(0);
p.time_constant = 1;
p.H = 0.0001;
p.tGo_initial_s = 1e-5;
p.tGo_min_s = 1e-5;
p.N_trials = 100000;
p.T_MAX = 10;
p.TIME_STEP = 0.1;

summary.t_final_s = zeros(p.T_MAX, 1);
summary.mean_miss = zeros(p.T_MAX, 1);
summary.std_miss = zeros(p.T_MAX, 1);
summary_idx = 1;



%% Monte-Carlo
for t_final = 0.02:p.TIME_STEP:p.T_MAX
    tGo_s = p.tGo_initial_s;
    log_step = 100 * p.H;
    log_stride = round(log_step/p.H);
    num_logs = floor((t_final - p.tGo_initial_s)/log_step) + 1;
    num_steps = floor(t_final / p.H + 1);
    log_idx = 1;

    log.time_s = zeros(num_logs,1);
    log.miss_maneuver_m = zeros(num_logs,1);
    log.miss_heading_m = zeros(num_logs,1);

    x = [0,0,1,0,0,0];

    %% Runge-Katta
    for step_idx = 1:num_steps
        if mod(step_idx-1, log_stride) == 0
            log.time_s(log_idx) = tGo_s;
            log.miss_maneuver_m(log_idx) = p.target_acceleration_mps2 * x(1);
            log.miss_heading_m(log_idx) = -p.missile_speed_mps * p.heading_error_rad * x(2);
            log.mudnt(log_idx) = sqrt((power(p.target_acceleration_mps2,2) * x(5) / tGo_s) ...
                - power(p.target_acceleration_mps2 * x(6) / tGo_s, 2));
            log.mean_miss_m(log_idx) = p.target_acceleration_mps2 * x(6) / tGo_s;
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

    time_gpu = gpuArray(log.time_s);
    miss_gpu = gpuArray(log.miss_maneuver_m);

    maneuver_time = t_final * rand(1, p.N_trials, "gpuArray");
    t_go_samples = max(t_final - maneuver_time, p.tGo_min_s);
    t_go_samples = min(t_go_samples, log.time_s(log_idx-1));

    miss_samples = interp1(time_gpu, miss_gpu, t_go_samples, "linear");

    summary.t_final_s(summary_idx) = t_final;
    summary.mean_miss(summary_idx) = gather(mean(miss_samples));
    summary.std_miss(summary_idx) = gather(std(miss_samples));
    summary_idx = summary_idx+1;
end

%% Results
results = table( ...
    summary.t_final_s, ...
    summary.mean_miss,...
    summary.std_miss, ...
    'VariableNames', {'time_s', 'mean_miss', 'std_miss'} ...
    );

fig = figure;
tiledlayout(2,1)

nexttile
plot(results.time_s, results.mean_miss, 'r+', log.time_s, log.mean_miss_m)
grid on
xlabel('Flight Time (s)')
ylabel('Mean Miss Distance (m)')

nexttile
plot(results.time_s, results.std_miss, 'r+', log.time_s, log.mudnt)
grid on
xlabel('Flight Time (s)')
ylabel('Standard Deviation of Miss Distance (m)')

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

check = savedata('Monte Carlo Single Lag Adjoint', results, p, fig);
if ~check
    disp("Error saving file")
end

%% Functions
function xd = adjoint_rhs(t, x, p)
t_go_s = max(t, p.tGo_min_s);

x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);

y1 = (x4-x2)/p.time_constant;
x1d = x2;
x2d = x3;
x3d = (y1*p.navigation_ratio)/t_go_s;
x4d = -y1;
x5d = power(x1,2);
x6d = x1;

xd = [x1d, x2d, x3d, x4d, x5d, x6d];
end
