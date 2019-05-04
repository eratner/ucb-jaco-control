global m1 m2 l1 l2 g q_des q_des_dot q_des_dd;
m1 = 1; m2 = 1; l1 = 1; l2 = 1; g = 9.81;
q_des = @(t) [exp(-t); sin(t)];
q_des_dot = @(t) [-exp(-t); cos(t)];
q_des_dd = @(t) [exp(-t); -sin(t)];

% feedback linearization plotting
figure(1);
title("Feedback Linearization (CTC)");
tspan = [0, 10];
x0 = [-0.2; 0.2; 0; 0];
f = @(t, x) arm_dynamics(t, x, @fb_linearization);
[t, s] = ode23(f, tspan, x0);
subplot(121);
plot(t, s(:,1));
hold on;
q1_des = q_des(t');
plot(t, q1_des(1,:));
legend("q1", "q1 desired");
ylim([-1.5, 1.5]);

subplot(122);
plot(t, s(:,2));
hold on;
q1_des = q_des(t');
plot(t, q1_des(2,:));
legend("q2", "q2 desired");
ylim([-1.5, 1.5]);

% sliding mode plotting
figure(2);
title("Sliding Mode controller");
tspan = [0, 10];
x0 = [-0.2; 0.2; 0; 0];
f = @(t, x) arm_dynamics(t, x, @sliding_mode);
[t, s] = ode23(f, tspan, x0);
subplot(121);
plot(t, s(:,1));
hold on;
q1_des = q_des(t');
plot(t, q1_des(1,:));
legend("q1", "q1 desired");
ylim([-1.5, 1.5]);

subplot(122);
plot(t, s(:,2));
hold on;
q1_des = q_des(t');
plot(t, q1_des(2,:));
legend("q2", "q2 desired");
ylim([-1.5, 1.5]);

function torque = fb_linearization(t, x, M, V, G, C)
    global q_des q_des_dot;
    Kp = 10*eye(2);
    Kd = 10*eye(2);
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
    
    q1 = x(1); q2 = x(2); q1dot = x(3); q2dot = x(4);
    
    M = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2), m2*l2^2 + m2*l1*l2*cos(q2);
         m2*l2^2 + m2*l1*l2*cos(q2)                 , m2*l2^2                  ];
    V = [-m2*l1*l2*(2*q1dot*q2dot + q2dot^2)*sin(q2);
         m2*l1*l2*q1dot^2*sin(q2)                   ];
    G = [(m1+m2)*g*l1*cos(q1) + m2*g*l2*cos(q1 + q2);
         m2*g*l2*cos(q1 + q2)                      ];
    C = [0, -m2*l1*l2*(2*q1dot + q2dot)*sin(q2);
         m2*l1*l2*q1dot*sin(q1), 0            ];
    N = V + G;
    control_torque = controller(t, x, M, V, G, C);
    qdd = inv(M) * (control_torque - N);
    dxdt = [q1dot;
            q2dot;
            qdd(1);
            qdd(2)];
end