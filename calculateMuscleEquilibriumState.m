% calculate muscle equilibrium state

T_G = 39.2236520436300; % theta_init = -0.05
% T_G = 54.8911465904957; % theta_init = -0.07
l_MTC = A0 + A1*theta_init;
momentArms = A1;
F_G = - momentArms^(-1) * T_G;
F_CE_rel = F_G / F_max;
F_SE = F_G;
T_M = - momentArms * F_SE;
l_SE = F_G / K_SE + l_SE_0;
l_CE_init = l_MTC - l_SE;
v_CE = 0.0;

l_CE_rel = l_CE_init / l_CE_opt;
F_isom_n = -a * l_CE_rel.^2 + 2*a.*l_CE_rel - a + 1;
q = F_CE_rel / F_isom_n;
rho = muscle_c * muscle_eta * (muscle_k - 1) ./ (muscle_k - l_CE_rel) .* l_CE_rel;
radiant = ((muscle_q0-q).*(q-1).^(-1));
STIM_init = rho.^(-1) .* (abs(radiant)).^(1/3) .* sign(radiant);
STIM_init = min([1 STIM_init]);
STIM_init = max([0 STIM_init]);

lambda_init = l_CE_init - 1 / K_l * STIM_init;
