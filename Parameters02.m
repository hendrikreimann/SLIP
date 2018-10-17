% biomechanics
l_foot = 0.1252;
h_com = 1;
h_head = 1.8;
r_body = 0.05;
m_body = 80;
theta_init = -0.05;


% delays
d_l_CE = 0.06;
d_v_CE = 0.03;
d_F_CE = 0.03;
d_theta = 0.06;
d_thetaDot = 0.12;
d_thetaTwoDot = 0.12;

% muscle-tendon complex
A0 = 0.2741;
A1 = -0.0627;
K_SE = 1.8e+05;
l_SE_0 = 0.2356;
l_CE_opt = 0.055;
F_max = 1.0e+04 * (0.5991 + 0.2996); % use soleus, but add the strength of the gastrocnemius
muscle_width = 0.56;
muscle_c = 1.3700e-04;
muscle_eta = 52700;
muscle_k = 2.9;
muscle_q0 = 0.005;
muscle_m = 11.3;
a_rel = 0.41;
b_rel = 5.2;
q_crit = 0.03;
a = 1 / muscle_width^2;
q_0 = 5e-03;

% muscle feedback
K_l = 1.0;
K_v = 0.3;
K_f = 0.0;

% higher feedback
alpha_theta = 0.0;
alpha_v = 3.0 * 5/9;
alpha_a = 0.3 * 5/9;
alpha_f = 0;
