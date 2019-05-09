global m1 m2 l1 l2 g q_des q_des_dot q_des_dd A B;
global m1_cont m2_cont l1_cont l2_cont;
global noisy disturbance constant_disturbance gaussian_disturbance saturation saturation_value num_trials;
global control_torques;
global previous_t error_integral; % for PID
m1 = 0.5; m2 = 0.5; l1 = 0.5; l2 = 0.5; g = 9.81;
m1_cont = m1; m2_cont = m2; l1_cont = l1; l2_cont = l2;
q_des = @(t) [exp(-t); sin(t)];
q_des_dot = @(t) [-exp(-t); cos(t)];
q_des_dd = @(t) [exp(-t); -sin(t)];
[A, B] = linearized_model();
num_trials = 1;
noisy = false;
saturation = true;
saturation_value = 30;
disturbance = 'none';
constant_disturbance = [1; 1];
gaussian_disturbance = @(t) [sin(t); sin(t)];
control_torques = [];
previous_t = 0; error_integral = zeros(2, 1);

figure(1);
plot_errors(@fb_linearization);

figure(2);
plot_errors(@sliding_mode);

figure(3);
plot_errors(@backstepping);

figure(4);
plot_errors_adaptive(@adaptive_control);

figure(5);
plot_errors(@pid);

function subs = subsample(list1, list2)
    step = floor(length(list2) / length(list1));
    % assumes list 1 shorter than list 2
    subs = [];
    for i=1:length(list1)
        subs = [subs, list2(:,step*i)];
    end
end

function Ts = settling_time(t, response)
    idx1 = find(response > .05, 1, 'last');
    if isempty(idx1)
        idx1 = 1;
    end
    idx2 = find(response < -0.05, 1, 'last');
    if isempty(idx2)
        idx2 = 1;
    end
    if idx1 == 1 && idx2 == 1
       Ts = t(end);
    else
       Ts = max(t(idx1), t(idx2)); 
    end
end

function os = overshoot(response)
    [peaks, ~] = findpeaks(response);
    os = max(0, peaks(1)-0) / 1 * 100;
end

function plot_errors(controller)
    global q_des q_des_dot;
    global m1 m2 l1 l2;
    global m1_cont m2_cont l1_cont l2_cont noisy num_trials;
    global control_torques;
    global previous_t error_integral;
    rng(0); % seed random number generator
    
    all_trajectories = [];
    settling_times = [];
    overshoots = [];
    tf = 10;
    tspan = 0:0.05:tf;
    x0 = [0; 1; 0; 0];
    for i=1:num_trials
        % set the parameters for the controller
        if noisy
            m1_cont = m1 + normrnd(0, 0.25);
            m2_cont = m2 + normrnd(0, 0.25);
            l1_cont = l1;
            l2_cont = l2;
        else
            m1_cont = m1;
            m2_cont = m2;
            l1_cont = l1;
            l2_cont = l2;
        end
        f = @(t, x) arm_dynamics(t, x, controller);
        [t, s] = ode23(f, tspan, x0);
        s = s - [q_des(t')', q_des_dot(t')'];
        controls = subsample(t, control_torques)';
        control_torques = [];
        s = [s, controls];
        Ts = [];
        os = [];
        for j=1:4
            Ts =  [Ts, settling_time(t, s(:,j))];
            os = [os, overshoot(s(:,j))];
        end
        settling_times = cat(1, settling_times, Ts);
        overshoots = cat(1, overshoots, os);
        all_trajectories = cat(3, all_trajectories, s);
        i
        error_integral = zeros(2, 1); previous_t = 0;
    end
    std_errors = std(all_trajectories, 0, 3) / sqrt(num_trials);
    s = mean(all_trajectories, 3);
    
    titles = ["q1 - q1 desired", "q2 - q2 desired", "q1dot - q1dot desired", "q2dot - q2dot desired", "control torque 1", "control torque 2"];
    nrows = 3; ncols = 2;
    avg_settling_times = mean(settling_times, 1);
    avg_overshoots = mean(overshoots, 1);
    for i=1:nrows*ncols
        subplot(nrows, ncols, i);
        upper_error_bar = s(:,i) + std_errors(:,i);
        lower_error_bar = s(:,i) - std_errors(:,i);
        plot(t, s(:,i));
        hold on;
        plot(t, zeros(length(t), 1), '--k');
        h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
        hold off;
        set(h, 'facealpha', 0.7);
        if i <= 4
            title(titles(i) + ", Ts: " + avg_settling_times(i) + ", %OS: " + avg_overshoots(i));
        else
            title(titles(i));
        end
    end
end

function plot_errors_adaptive(controller)
    global q_des q_des_dot;
    global m1 m2 l1 l2;
    global m1_cont m2_cont l1_cont l2_cont noisy num_trials;
    global control_torques;
    rng(0); % seed random number generator
    all_trajectories = [];
    settling_times = [];
    overshoots = [];
    tf = 10;
    tspan = 0:0.05:tf;
    for i=1:num_trials
        % set the parameters for the controller
        if noisy
            m1_cont = m1 + normrnd(0, 0.25);
            m2_cont = m2 + normrnd(0, 0.25);
            l1_cont = l1;
            l2_cont = l2; 
        else
            m1_cont = m1;
            m2_cont = m2;
            l1_cont = l1;
            l2_cont = l2;
        end
        x0 = [0; 1; 0; 0; m1_cont; m2_cont];
        f = @(t, x) arm_dynamics_adaptive(t, x, controller);
        [t, s] = ode23(f, tspan, x0);
        s = s - [q_des(t')', q_des_dot(t')', m1 * ones(length(t), 1), m2 * ones(length(t), 1)];
        controls = subsample(t, control_torques)';
        control_torques = [];
        s = [s, controls];
        Ts = [];
        os = [];
        for j=1:4
            Ts =  [Ts, settling_time(t, s(:,j))];
            os = [os, overshoot(s(:,j))];
        end
        settling_times = cat(1, settling_times, Ts);
        overshoots = cat(1, overshoots, os);
        all_trajectories = cat(3, all_trajectories, s);
        i
    end
    std_errors = std(all_trajectories, 0, 3) / sqrt(num_trials);
    s = mean(all_trajectories, 3);
    
    titles = ["q1 - q1 desired", "q2 - q2 desired", "q1dot - q1dot desired", "q2dot - q2dot desired", ...
        "m1 estimate - m1", "m2 estimate - m2", "control torque 1", "control torque 2"];
    nrows = 4; ncols = 2;
    avg_settling_times = mean(settling_times, 1);
    avg_overshoots = mean(overshoots, 1);
    for i=1:nrows*ncols
        subplot(nrows, ncols, i);
        upper_error_bar = s(:,i) + std_errors(:,i);
        lower_error_bar = s(:,i) - std_errors(:,i);
        plot(t, s(:,i));
        hold on;
        plot(t, zeros(length(t), 1), '--k');
        h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
        hold off;
        set(h, 'facealpha', 0.7);
        if i <= 4
            title(titles(i) + ", Ts: " + avg_settling_times(i) + ", %OS: " + avg_overshoots(i));
        else
            title(titles(i));
        end
    end    
end

function torque = linearized_control(t, x, M, V, G, C)
    global A B;
    
    K = place(A, B, [-1, -1, -2, -2]);
    torque = -K*x;
end

function [A, B] = linearized_model()
    global m1 m2 l1 l2 g;
    syms q1 q2 q1dot q2dot;
%     q1 = x(1); q2 = x(2); q1dot = x(3); q2dot = x(4);
    
    M = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2), m2*l2^2 + m2*l1*l2*cos(q2);
         m2*l2^2 + m2*l1*l2*cos(q2)                 , m2*l2^2                  ];
    V = [-m2*l1*l2*(2*q1dot*q2dot + q2dot^2)*sin(q2);
         m2*l1*l2*q1dot^2*sin(q2)                   ];
    G = [(m1+m2)*g*l1*cos(q1) + m2*g*l2*cos(q1 + q2);
         m2*g*l2*cos(q1 + q2)                      ];
    N = V + G;
    
    f = [q1dot;
         q2dot;
         -inv(M)*N];
    
    A = subs(jacobian(f, [q1; q2; q1dot; q2dot]), [q1; q2; q1dot; q2dot], [0;0;0;0]);
    B = subs([0 0; 0 0; inv(M)], [q1; q2; q1dot; q2dot], [0;0;0;0]);
    
end

function torque = pid(t, x, M, V, G, C)
    global q_des q_des_dot;
    global previous_t error_integral;
    q = [x(1); x(2)]; qdot = [x(3); x(4)];
    Kp = 60 * eye(2);
    Ki = 10 * eye(2);
    Kd = 30 * eye(2);
    
    error = q - q_des(t);
    errordot = qdot - q_des_dot(t);
    error_integral = error_integral + error*(t - previous_t);
    previous_t = t;
    
    torque = -Kp*error - Ki*error_integral - Kd*errordot;
end

function [torque, param_adapt] = adaptive_control(t, x, M, V, G, C)
    global q_des q_des_dot q_des_dd l1_cont l2_cont g;
    q1 = x(1); q2 = x(2); q1dot = x(3); q2dot = x(4);
    q = [q1; q2]; qdot = [q1dot; q2dot];
    qbar = q - q_des(t);
    qbar_dot = qdot - q_des_dot(t);
    Gamma = 1 * eye(2);
    Lambda = 10 * eye(2);
    Kd = 10 * eye(2);
    s = qbar_dot + Lambda*qbar;
    qrdot = q_des_dot(t) - Lambda*qbar;
    qrdd = q_des_dd(t) - Lambda*qbar_dot;
    
    torque = M*qrdd + C*qrdot + G - Kd*s;
    
    qdd = inv(M)*(torque - V - G);
    q1dd = qdd(1); q2dd = qdd(2);
    q1rdot = qrdot(1); q2rdot = qrdot(2);
    q1rdd = qrdd(1); q2rdd = qrdd(2);
%     regression matrix for 2dof arm
    Y1 = [l1_cont^2*q1rdd, (l1_cont^2+l2_cont^2 + 2*l1_cont*l2_cont*cos(q2))*q1rdd + (l2_cont^2 + l1_cont*l2_cont*cos(q2))*q2rdd;
          0, (l2_cont^2 + l1_cont*l2_cont*cos(q2))*q1rdd + l2_cont^2*q2rdd];
    Y2 = [0, -l1_cont*l2_cont*(2*q1dot + q2dot)*sin(q2)*q2rdot;
          0, l1_cont*l2_cont*q1dot*sin(q1)*q1rdot];
    Y3 = [g*l1_cont*cos(q1), g*l1_cont*cos(q1) + g*l2_cont*cos(q1 + q2);
          0, g*l2_cont*cos(q1 + q2)];
    Y = Y1 + Y2 + Y3;
        
    % adaptation law
    param_adapt = -inv(Gamma)*Y'*s;
end

function torque = backstepping(t, x, M, V, G, C)
    global q_des q_des_dot q_des_dd;
    q = [x(1); x(2)];
    qdot = [x(3); x(4)];
    Lambda = 10 * eye(2);
    K1 = 10 * eye(2);
    z1 = q - q_des(t);
    qrdot = q_des_dot(t) - Lambda*z1;
    
    torque = M*q_des_dd(t) + C*qrdot - z1 - K1*(qdot - qrdot) + G;
end

function torque = fb_linearization(t, x, M, V, G, C)
    global q_des q_des_dot;
    Kp = 200*eye(2);
    Kd = 5*eye(2);
    q = [x(1); x(2)];
    qdot = [x(3); x(4)];
    qbar = q - q_des(t);
    qbar_dot = qdot - q_des_dot(t);
    
    u = -Kp * qbar - Kd*qbar_dot;
    torque = V + G + M*u;
end

function torque = sliding_mode(t, x, M, V, G, C)
    global q_des q_des_dot q_des_dd;
    q = [x(1); x(2)];
    qdot = [x(3); x(4)];
    qbar = q - q_des(t);
    qbar_dot = qdot - q_des_dot(t);
    Lambda = 4*eye(2);
    s = qbar_dot + Lambda*qbar;
    K = 4*eye(2);
    
    torque = V + G + M*q_des_dd(t) - M*Lambda*qbar_dot - C*s - K*s;
end

function torque = dummy_controller(t, x, M, V, G, C)
    torque = [0; 0];
end

function dxdt = arm_dynamics(t, x, controller)
    global m1 m2 l1 l2 g;
    global m1_cont m2_cont l1_cont l2_cont;
    global control_torques;
    global disturbance constant_disturbance gaussian_disturbance saturation saturation_value;
    
    q1 = x(1); q2 = x(2); q1dot = x(3); q2dot = x(4);
    
    M_cont = [(m1_cont+m2_cont)*l1_cont^2 + m2_cont*l2_cont^2 + 2*m2_cont*l1_cont*l2_cont*cos(q2), m2_cont*l2_cont^2 + m2_cont*l1_cont*l2_cont*cos(q2);
         m2_cont*l2_cont^2 + m2_cont*l1_cont*l2_cont*cos(q2)                 , m2_cont*l2_cont^2                  ];
    V_cont = [-m2_cont*l1_cont*l2_cont*(2*q1dot*q2dot + q2dot^2)*sin(q2);
         m2_cont*l1_cont*l2_cont*q1dot^2*sin(q2)                   ];
    G_cont = [(m1_cont+m2_cont)*g*l1_cont*cos(q1) + m2_cont*g*l2_cont*cos(q1 + q2);
         m2_cont*g*l2_cont*cos(q1 + q2)                      ];
    C_cont = [0, -m2_cont*l1_cont*l2_cont*(2*q1dot + q2dot)*sin(q2);
         m2_cont*l1_cont*l2_cont*q1dot*sin(q1), 0            ];
    
    M = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2), m2*l2^2 + m2*l1*l2*cos(q2);
         m2*l2^2 + m2*l1*l2*cos(q2)                 , m2*l2^2                  ];
    V = [-m2*l1*l2*(2*q1dot*q2dot + q2dot^2)*sin(q2);
         m2*l1*l2*q1dot^2*sin(q2)                   ];
    G = [(m1+m2)*g*l1*cos(q1) + m2*g*l2*cos(q1 + q2);
         m2*g*l2*cos(q1 + q2)                      ];
    C = [0, -m2*l1*l2*(2*q1dot + q2dot)*sin(q2);
         m2*l1*l2*q1dot*sin(q1), 0            ];
    N = V + G;
    
    control_torque = controller(t, x, M_cont, V_cont, G_cont, C_cont);
    if disturbance == "constant"
        control_torque = control_torque + constant_disturbance;
    elseif disturbance == "gaussian"
        control_torque = control_torque + gaussian_disturbance(t);
    end
    if saturation
        control_torque = min(control_torque, [saturation_value; saturation_value]);
        control_torque = max(control_torque, [-saturation_value; -saturation_value]);
    end
    control_torques = [control_torques, control_torque]; % save current control input
    qdd = inv(M) * (control_torque - N);
    dxdt = [q1dot;
            q2dot;
            qdd(1);
            qdd(2)];
end

function dxdt = arm_dynamics_adaptive(t, x, controller)
    global m1 m2 l1 l2 g;
    global m1_cont m2_cont l1_cont l2_cont;
    global control_torques;
    global disturbance constant_disturbance gaussian_disturbance saturation saturation_value;
    
    q1 = x(1); q2 = x(2); q1dot = x(3); q2dot = x(4);
    m1_cont = x(5); m2_cont = x(6);
    
    M_cont = [(m1_cont+m2_cont)*l1_cont^2 + m2_cont*l2_cont^2 + 2*m2_cont*l1_cont*l2_cont*cos(q2), m2_cont*l2_cont^2 + m2_cont*l1_cont*l2_cont*cos(q2);
         m2_cont*l2_cont^2 + m2_cont*l1_cont*l2_cont*cos(q2)                 , m2_cont*l2_cont^2                  ];
    V_cont = [-m2_cont*l1_cont*l2_cont*(2*q1dot*q2dot + q2dot^2)*sin(q2);
         m2_cont*l1_cont*l2_cont*q1dot^2*sin(q2)                   ];
    G_cont = [(m1_cont+m2_cont)*g*l1_cont*cos(q1) + m2_cont*g*l2_cont*cos(q1 + q2);
         m2_cont*g*l2_cont*cos(q1 + q2)                      ];
    C_cont = [0, -m2_cont*l1_cont*l2_cont*(2*q1dot + q2dot)*sin(q2);
         m2_cont*l1_cont*l2_cont*q1dot*sin(q1), 0            ];
    
    M = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2), m2*l2^2 + m2*l1*l2*cos(q2);
         m2*l2^2 + m2*l1*l2*cos(q2)                 , m2*l2^2                  ];
    V = [-m2*l1*l2*(2*q1dot*q2dot + q2dot^2)*sin(q2);
         m2*l1*l2*q1dot^2*sin(q2)                   ];
    G = [(m1+m2)*g*l1*cos(q1) + m2*g*l2*cos(q1 + q2);
         m2*g*l2*cos(q1 + q2)                      ];
    C = [0, -m2*l1*l2*(2*q1dot + q2dot)*sin(q2);
         m2*l1*l2*q1dot*sin(q1), 0            ];
    N = V + G;
    
    [control_torque, param_adapt] = controller(t, x, M_cont, V_cont, G_cont, C_cont);
    if disturbance == "constant"
        control_torque = control_torque + constant_disturbance;
    elseif disturbance == "gaussian"
        control_torque = control_torque + gaussian_disturbance(t);
    end
    if saturation
        control_torque = min(control_torque, [saturation_value; saturation_value]);
        control_torque = max(control_torque, [-saturation_value; -saturation_value]);
    end
    control_torques = cat(2, control_torques, control_torque); % save current control input
    qdd = inv(M) * (control_torque - N);
    dxdt = [q1dot;
            q2dot;
            qdd(1);
            qdd(2);
            param_adapt(1);
            param_adapt(2)];
end