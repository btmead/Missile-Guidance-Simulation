clc
clear
close all

%% Parameters
p.G = 9.81;
p.nav_ratio = 3;
p.time_constant = 1;
p.t_final_s = 10;
p.t_int = 0.5;
p.target_accel_mps2 = 3 * p.G;
p.t_initial_s = 1e-5;
p.h = 0.0001;
p.miss = 0; %If looking for miss adjoint: 1, for acceleration adjoint: 0

num_steps = floor((p.t_final_s - (p.t_initial_s + p.t_int)) / p.h); 
t = p.t_initial_s + p.t_int;
t_go = p.t_final_s - t;


if p.miss == 1
    x = [0, 0, 1, 0, 0];
    yaxis = "Miss Distance (m)";
else
    x3_init = p.nav_ratio / (p.time_constant * p.t_int);
    x4_init = -1 / p.time_constant;
    yaxis = "Standard deviation of n_L (g)";

    x = [0, 0, x3_init, x4_init, 0];
end

%% Runge-Katta
for step_idx = 1:num_steps

    k1 = adjoint_rhs(p, x,                  t            );
    k2 = adjoint_rhs(p, x + k1 * p.h / 2,   t + p.h / 2  );
    k3 = adjoint_rhs(p, x + k2 * p.h / 2,   t + p.h / 2  );
    k4 = adjoint_rhs(p, x + k3 * p.h,       t + p.h      );

    x = x + p.h / 6 * (k1 + 2*k2 + 2*k3 + k4);

    log.time_s(step_idx) = t;
    log.mudnt(step_idx) = sqrt(x(5) * (power(p.target_accel_mps2,2) / t));
    if p.miss == 0
        log.mudnt(step_idx) = log.mudnt(step_idx) / p.G;
    end

    t = t + p.h;
    t_go = p.t_final_s - t;
end

%% Results

results = results_table(log);

fig = figure;
plot(log.time_s, log.mudnt)
xlabel('Time (s)')
ylabel(yaxis)
grid on

runID = run_id(p);
check = savedata('Acceleration & Miss Distance Adjoint', results, fig, runID);



%% Functions
function[xd] = adjoint_rhs(p,x,t)
    
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);
    
    y1 = (x4 - x2) / p.time_constant;
    x1d = x2;
    x2d = x3;
    x3d = y1 * (p.nav_ratio / t);
    x4d = -y1;
    x5d = power(x1, 2);

    xd = [x1d, x2d, x3d, x4d, x5d];
end

