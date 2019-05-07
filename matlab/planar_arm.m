global m1 m2 l1 l2 g q_des q_des_dot q_des_dd A B;
global m1_cont m2_cont l1_cont l2_cont noisy;
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;
m1_cont = m1; m2_cont = m2; l1_cont = l1; l2_cont = l2;
% q_des = @(t) [sin(t); exp(-t)];
% q_des_dot = @(t) [cos(t); -exp(-t)];
% q_des_dd = @(t) [-sin(t); exp(-t)];
q_des = @(t) [exp(-t); sin(t)];
q_des_dot = @(t) [-exp(-t); cos(t)];
q_des_dd = @(t) [exp(-t); -sin(t)];
[A, B] = linearized_model();
noisy = true;

% figure(1);
% plot_errors(@fb_linearization);
% 
% figure(2);
% plot_errors(@sliding_mode);
% 
% figure(3);
% plot_errors(@backstepping);
% 
% figure(4);
plot_errors_adaptive(@adaptive_control);

function plot_errors(controller)
    global q_des q_des_dot;
    global m1 m2 l1 l2;
    global m1_cont m2_cont l1_cont l2_cont noisy;
    
    all_trajectories = [];
    tspan = 0:0.05:10;
    x0 = [1; 0; 0; 0];
    num_trials = 100;
    for i=1:num_trials
        % set the parameters for the controller
        if noisy
            m1_cont = m1 + normrnd(0, 0.1);
            m2_cont = m2 + normrnd(0, 0.1);
            l1_cont = l1; %+ normrnd(0, 0.1);
            l2_cont = l2; %+ normrnd(0, 0.1);
        else
            m1_cont = m1;
            m2_cont = m2;
            l1_cont = l1;
            l2_cont = l2;
        end
        
        f = @(t, x) arm_dynamics(t, x, controller);
        [t, s] = ode23(f, tspan, x0);
        all_trajectories = cat(3, all_trajectories, s);
    end
    std_errors = std(all_trajectories, 0, 3) / sqrt(num_trials);
    s = mean(all_trajectories, 3);
    
    subplot(221);
    q1_des = q_des(t')';
    error = s(:,1) - q1_des(:,1);
    upper_error_bar = error + std_errors(:,1);
    lower_error_bar = error - std_errors(:,1);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q1 - q1 desired");

    subplot(222);
    q2_des = q_des(t')';
    error = s(:,2) - q2_des(:,2);
    upper_error_bar = error + std_errors(:,2);
    lower_error_bar = error - std_errors(:,2);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q2 - q2 desired");
    
    subplot(223);
    q1_des_dot = q_des_dot(t')';
    error = s(:,3) - q1_des_dot(:,1);
    upper_error_bar = error + std_errors(:,3);
    lower_error_bar = error - std_errors(:,3);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q1dot - q1dot desired");

    subplot(224);
    q2_des_dot = q_des_dot(t')';
    error = s(:,4) - q2_des_dot(:,2);
    upper_error_bar = error + std_errors(:,4);
    lower_error_bar = error - std_errors(:,4);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q2dot - q2dot desired");
end

function plot_errors_adaptive(controller)
    global q_des q_des_dot;
    global m1 m2 l1 l2;
    global m1_cont m2_cont l1_cont l2_cont noisy;
    
    all_trajectories = [];
    tspan = 0:0.05:10;
    x0 = [1; 0; 0; 0; m1-0.3; m2-0.3];
    num_trials = 50;
    for i=1:num_trials
        % set the parameters for the controller
        if noisy
            m1_cont = m1 + normrnd(0, 0.5);
            m2_cont = m2 + normrnd(0, 0.5);
            l1_cont = l1; %+ normrnd(0, 0.1);
            l2_cont = l2; %+ normrnd(0, 0.1);
        else
            m1_cont = m1;
            m2_cont = m2;
            l1_cont = l1;
            l2_cont = l2;
        end
        
        f = @(t, x) arm_dynamics_adaptive(t, x, controller);
        [t, s] = ode23(f, tspan, x0);
        all_trajectories = cat(3, all_trajectories, s);
    end
    std_errors = std(all_trajectories, 0, 3) / sqrt(num_trials);
    s = mean(all_trajectories, 3);
    
    subplot(321);
    q1_des = q_des(t')';
    error = s(:,1) - q1_des(:,1);
    upper_error_bar = error + std_errors(:,1);
    lower_error_bar = error - std_errors(:,1);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q1 - q1 desired");

    subplot(322);
    q2_des = q_des(t')';
    error = s(:,2) - q2_des(:,2);
    upper_error_bar = error + std_errors(:,2);
    lower_error_bar = error - std_errors(:,2);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q2 - q2 desired");
    
    subplot(323);
    q1_des_dot = q_des_dot(t')';
    error = s(:,3) - q1_des_dot(:,1);
    upper_error_bar = error + std_errors(:,3);
    lower_error_bar = error - std_errors(:,3);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q1dot - q1dot desired");

    subplot(324);
    q2_des_dot = q_des_dot(t')';
    error = s(:,4) - q2_des_dot(:,2);
    upper_error_bar = error + std_errors(:,4);
    lower_error_bar = error - std_errors(:,4);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("q2dot - q2dot desired");
    
    subplot(325);
    error = s(:,5) - m1 * ones(length(t));
    upper_error_bar = error + std_errors(:,5);
    lower_error_bar = error - std_errors(:,5);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("m1 estimate - m1");
    
    subplot(326);
    error = s(:,6) - m1 * ones(length(t));
    upper_error_bar = error + std_errors(:,6);
    lower_error_bar = error - std_errors(:,6);
    plot(t, error);
    hold on;
    h = fill([t; flipud(t)], [lower_error_bar; flipud(upper_error_bar)], [0.9 0.9 0.9], 'linestyle','none');
    hold off;
    set(h, 'facealpha', 0.7);
    title("m2 estimate - m2");
    
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
    % regression matrix for 2dof arm
    Y = [l1_cont^2*q1dd + l1_cont*g*cos(q1), l2_cont^2*(2*q1dd + q2dd) + l1_cont^2*q1dd - l1_cont*l2_cont*sin(q2)*q2dot^2 - 2*l1_cont*l2_cont*sin(q2)*q1dot*q2dot + l2_cont*g*cos(q1 + q2) + l1_cont*g*cos(q1);
         0, l1_cont*l2_cont*cos(q2)*q1dd + l1_cont*l2_cont*sin(q1)*q1dot^2 + l2_cont*g*cos(q1+q2) + l2_cont^2*(q1dd + q2dd)];
    
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
    qdd = inv(M) * (control_torque - N);
    dxdt = [q1dot;
            q2dot;
            qdd(1);
            qdd(2)];
end

function dxdt = arm_dynamics_adaptive(t, x, controller)
    global m1 m2 l1 l2 g;
    global m1_cont m2_cont l1_cont l2_cont;
    
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
    qdd = inv(M) * (control_torque - N);
    dxdt = [q1dot;
            q2dot;
            qdd(1);
            qdd(2);
            param_adapt(1);
            param_adapt(2)];
end