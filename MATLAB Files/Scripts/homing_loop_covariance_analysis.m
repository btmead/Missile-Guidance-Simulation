clc
clear
close all

%% Parameters
p.G = 9.81;
p.nav_ratio = 3;
p.time_constant = 1;
p.t_final_s = 10;
p.closing_vel_mps = 1219;
p.target_accel_mps2 = 3 * p.G;
p.t_initial_s = 1e-5;
p.h = 0.0001;


t_s = p.t_initial_s;
t_go_s = p.t_final_s - t_s;

Q = zeros([4,4]);
Q(3,3) = power(p.target_accel_mps2,2) / p.t_final_s;

X = zeros([4,4]);

num_steps = floor((p.t_final_s - p.t_initial_s) / p.h);
log_idx = 1;


%% Adjoint
for step_idx = 1:num_steps
    k1 = adjoint_rhs(p, Q, X,               t_s         );
    k2 = adjoint_rhs(p, Q, X + k1 * p.h/2,  t_s + p.h/2 );
    k3 = adjoint_rhs(p, Q, X + k2 * p.h/2,  t_s + p.h/2 );
    k4 = adjoint_rhs(p, Q, X + k3 * p.h,    t_s + p.h   );

    X = X + p.h/6 * (k1 + 2 * k2 + 2 * k3 + k4);    

    t_s = t_s + p.h;

    if rem(step_idx, 10) == 0
        A = F_time(p.t_final_s - t_s,p);
        A = -A(2,:);
        A(1,3) = 0;
        log.time(log_idx) = t_s;
        log.miss_distance(log_idx) = sqrt(X(1,1));
        log.missile_acceleration(log_idx) = sqrt(A * X * A') / p.G;

        log_idx = log_idx + 1;
    end
end


%% Results & Analysis
fig = figure;
tiledlayout(2,1)

nexttile
plot(log.time, log.miss_distance)
xlabel('Time (s)')
ylabel('Standard Deviation of y (m)')
grid on

nexttile
plot(log.time, log.missile_acceleration)
xlabel('Time (s)')
ylabel('Standard Deviation of Acceleration (g)')
ylim([0 15])
grid on


fprintf("Miss Distance: %.2f", sqrt(X(1,1)));

results = results_table(log);
runID = run_id(p);
check = savedata("Homing Loop Covariance Analysis", results, fig, runID);

function Xd = adjoint_rhs(p, Q, X, t)
    t = p.t_final_s - t;
    F = F_time(t, p);
    Xd = (F * X) + (F * X)' + Q;
end

function F_new = F_time(t, p)
    F_new = [0 1 0 0
        (-p.nav_ratio / (p.time_constant * t)) 0 1 (p.nav_ratio * p.closing_vel_mps / p.time_constant)
        0 0 0 0
        (1/(p.time_constant * p.closing_vel_mps * t)) 0 0 (-1 / p.time_constant)];
end