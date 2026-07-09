clc
clear
close all

%rng(1); %Seed number (use for debugging)

%% Parameters
p.tau_s = 0.2;
p.spectral_density = 1;
p.integration_step_s = 0.01;
p.t_final_s = 100;
p.sigma = sqrt(p.spectral_density / p.integration_step_s);

t = 0;
y = 0;
h = p.integration_step_s;
num_steps = floor(p.t_final_s / h) + 1;
sigma_lpf = 0;
extremes = 0;

log.time_s = zeros(num_steps, 1);
log.y_value = zeros(num_steps, 1);
log.x_value = zeros(num_steps, 1);
log.sigma_value = zeros(num_steps, 1);

%% Integration
for step_idx = 1:num_steps
    x = p.sigma  * randn;
    sigma_lpf = sqrt(p.spectral_density * (1 - exp(-2 * t / p.tau_s)) ...
        / (2 * p.tau_s));

    log.time_s(step_idx) = t;
    log.y_value(step_idx) = y;
    log.x_value(step_idx) = x;
    log.sigma_value(step_idx) = sigma_lpf;

    if y > sigma_lpf || y < -sigma_lpf
        extremes = extremes + 1;
    end
    
    k1 = low_pass_rhs(y,             x, p);
    k2 = low_pass_rhs(y + k1 * h/2,  x, p);
    k3 = low_pass_rhs(y + k2 * h/2,  x, p);
    k4 = low_pass_rhs(y + k3 * h,    x, p);

    y = y + h/6 * (k1 + 2 * k2 + 2 * k3 + k4);
    
    t = t + h;
end

%% Results & Figure
percentage_extremes = (1 - (extremes / num_steps)) * 100;
xmax_graph = max(log.x_value) + 3;

fig = figure;
tiledlayout(2,1)

nexttile
plot(log.time_s, log.y_value, ...
    log.time_s, log.sigma_value, ...
    log.time_s, -log.sigma_value ...
    );
grid on
title('Simulation of low-pass filter driven by white noise')
xlabel('Time (s)')
ylabel('y')
xlim([0 p.t_final_s]);
ylim([-xmax_graph xmax_graph]);

txt = sprintf([ ...
    'Percentage inside of \\sigma_{lpf}: %.2f%% \n' ...
    '\\sigma Parameter: %.2f'], percentage_extremes, p.sigma);
text(0.98, 0.98, txt, ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', 'white', ...
    'EdgeColor', 'black', ...
    'Interpreter', 'tex');

nexttile
plot(log.time_s, log.x_value)
grid on
xlabel('Time (s)')
ylabel('x')
xlim([0 p.t_final_s]);
ylim([-xmax_graph xmax_graph]);

results = table( ...
    log.time_s, ...
    log.y_value, ...
    log.sigma_value, ...
    'VariableNames', {'Time (s)', 'y Value', 'Sigma Value'} ...
    );

runID = run_id(p);
check = savedata('Low-pass Filter', results, fig, runID);
if ~check
    disp('Error saving file')
end

%% Functions
function yd = low_pass_rhs(y, x, p)
    yd = (x - y) / p.tau_s;
end
