clc
clear
close all

%% Parameters
G = 9.81;

p.seed = 2;
p.N = 3;
p.missile_vel = 1200;
p.target_vel = 800;
p.heading_err = deg2rad(10);
p.missile_pos = [2000; 100];
p.target_pos = [-7000; -10000];
p.h = 0.001;
p.beta = deg2rad(-26);
p.missile_max_accel = 30 * G;

rng(p.seed, "twister");
t = 0;
h = p.h;
step_idx = 1;
log_idx = 1;
log_int = 20;

rm = p.missile_pos;
rt = p.target_pos;
r = rt - rm;
lambda = atan2(r(2), r(1));
lead = asin(p.target_vel / p.missile_vel * sin(p.beta - lambda));
vm = [p.missile_vel * cos(lambda + p.heading_err + lead); p.missile_vel * sin(lambda + p.heading_err + lead)];
vt = [p.target_vel * cos(p.beta); p.target_vel * sin(p.beta)];
at = [0; 0];

%% RK4 Integration
state = [rm; vm; rt; vt];
vc = closing_velocity(state);

logging.time(log_idx) = t;
logging.missile_x(log_idx) = state(1);
logging.missile_y(log_idx) = state(2);
logging.target_x(log_idx) = state(5);
logging.target_y(log_idx) = state(6);
logging.closing_velocity(log_idx) = vc;
logging.missile_acceleration(log_idx) = NaN;
logging.target_acceleration(log_idx) = NaN;
log_idx = log_idx + 1;

while vc > 0 && t < 60
    at = rand_at(at, p, state);
    k1 = missile_dynamics(at, p, state,             t       );
    k2 = missile_dynamics(at, p, state + k1 * h/2,  t + h/2 );
    k3 = missile_dynamics(at, p, state + k2 * h/2,  t + h/2 );
    k4 = missile_dynamics(at, p, state + k3 * h,    t + h   );
    state_dot = (k1 + 2*k2 + 2*k3 + k4) / 6;
    state = state + h * state_dot;

    [vc, r] = closing_velocity(state);
    t = t + h;

    if rem(step_idx, log_int) == 0
        logging.time(log_idx) = t;
        logging.missile_x(log_idx) = state(1);
        logging.missile_y(log_idx) = state(2);
        logging.target_x(log_idx) = state(5);
        logging.target_y(log_idx) = state(6);
        logging.closing_velocity(log_idx) = vc;
        logging.missile_acceleration(log_idx) = norm(state_dot(3:4));
        logging.target_acceleration(log_idx) = norm(state_dot(7:8));

        log_idx = log_idx + 1;
    end
   
    step_idx = step_idx + 1;
    
    if norm(r) < 100
        log_int = 1;
    end
end


fprintf("Miss Distance: %.2f \n", abs(norm(r)));
fprintf("Time of engagement: %.2f \n", t);

%% Plotting
xmax = max(max(logging.missile_x), max(logging.target_x)) + 150;
ymax = max(max(logging.missile_y), max(logging.target_y)) + 150;
xmin = min(0, min(min(logging.missile_x), min(logging.target_x)) - 150);
ymin = min(0, min(min(logging.missile_y), min(logging.target_y)) - 150);

fig = figure;
tiledlayout(2, 1)

nexttile
plot(logging.missile_x, logging.missile_y, "Color", "red")
hold on
plot(logging.target_x, logging.target_y, "Color", "blue")
xlabel("X Direction (m)")
ylabel("Y Direction (m)")
axis equal
axis([xmin xmax ymin ymax])
grid on

nexttile
plot(logging.time, logging.missile_acceleration, "Color", "red")
hold on
plot(logging.time, logging.target_acceleration, "Color", "blue")
xlabel('Time(s)')
ylabel('Acceleration (m/s^2)')
ylim([0 p.missile_max_accel])
grid on

results = results_table(logging);
runID = run_id(p);
check = savedata('Proportional Navigation Homing Loop', results, fig, runID);
if check == false
    disp("Error saving results")
end

%% Functions
function state_dot = missile_dynamics(at, p, state, t)
rm = state(1:2);
vm = state(3:4);
rt = state(5:6);
vt = state(7:8);


r = rt - rm;
lambda = atan2(r(2), r(1));
v_rel = vt - vm;
vc = closing_velocity(state);

lambda_dot = (r(1) * v_rel(2) - r(2) * v_rel(1)) / power(norm(r),2);

nc = p.N * vc * lambda_dot;

ax = -nc * sin(lambda);
ay = nc * cos(lambda);


state_dot = [   
                vm(1);
                vm(2);
                ax;
                ay;
                vt(1);
                vt(2);
                at
            ];
end


function [vc,r] = closing_velocity(state)
rm = state(1:2);
vm = state(3:4);
rt = state(5:6);
vt = state(7:8);

r = rt - rm;
v_rel = vt - vm;
r_dot = dot(r, v_rel) / norm(r);
vc = -r_dot;
end

function target_acceleration = rand_at(at, p, state)

vt = state(7:8);
beta = atan2(vt(2), vt(1));
tau = 0.2;
alpha = exp(-p.h / tau);
sign = randi([0 1]) * 2 - 1;

magnitude = 9 * 9.81 * rand;
direction = beta + sign * pi / 2;

acceleration = magnitude * [
    cos(direction);
    sin(direction)
    ];

if norm(at) == 0
    target_acceleration = acceleration;
else
    target_acceleration = alpha * at + (1 - alpha) * acceleration;
end
end