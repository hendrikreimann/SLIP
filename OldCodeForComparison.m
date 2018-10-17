function ...
  [ ...
    stance_time, ...
    theta_history, ...
    T_M_history, ...
    l_CE_history, ...
    thetaDot_history ...
  ] ...
  = singleHillMuscleQuietStance ...
  ( ...
    random_number_seed, ...
    total_time, ...
    K_l, ...
    K_v, ...
    K_f, ...
    alpha_theta, ...
    alpha_v, ...
    alpha_a, ...
    alpha_f, ...
    motor_noise_strength, ...
    useInitialVelocityPerturbation ...
  )

    %% set unpassed parameters
    if nargin < 1; random_number_seed = 0; end
    if nargin < 2; total_time = 30.0; end
    if nargin < 3; K_l = 1.0; end
    if nargin < 4; K_v = 0.3; end
    if nargin < 5; K_f = 0.0; end
    if nargin < 6; alpha_theta = 0.0; end
    if nargin < 7; alpha_v = 3.0; end
    if nargin < 8; alpha_a = 0.3; end
    if nargin < 9; alpha_f = 0; end
    if nargin < 10; motor_noise_strength = 0; end
    if nargin < 11; useInitialVelocityPerturbation = 0; end
    if nargin >= 10; verbose = 0; else verbose = 1; end
    
    %% stable configuration, central feedback
    if nargin == 0
        random_number_seed = 0;
        total_time = 5;
        K_l = 1.0;
        K_v = 0.3;
        K_f = 0.0;
        alpha_theta = 0.0;
        alpha_v = 3.0 * 5/9;
        alpha_a = 0.3 * 5/9;
        alpha_f = 0;
        motor_noise_strength = 0.0001;
        useInitialVelocityPerturbation = 0;
    end
    
    % try out stuff
%     if nargin == 0
%         random_number_seed = 0;
%         total_time = 3;
%         K_l = 1.0;
%         K_v = 0.3;
%         K_f = 0.0;
%         alpha_theta = 0.0;
%         alpha_v = 0;
%         alpha_a = 0;
%         alpha_f = 1e-4;
%         motor_noise_strength = 0.0;
%         useInitialVelocityPerturbation = 1;
%     end
    
    initial_angle_perturbation = 0.0;
    if useInitialVelocityPerturbation
        initial_velocity_perturbation = -0.0046;
    else
        initial_velocity_perturbation = 0;
    end

    eulerStep = 0.001;
    time = eulerStep : eulerStep : total_time;

    show_plots = verbose;
%     show_plots = 1;

    %% set physiological parameters
    A1 = -0.0627;
    A0 = 0.2741;
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
    ankle_torque_threshold = 172.6;
%     passive_damping_gain = 5.73;
    passive_damping_gain = 0; % CHANGED
    
    %% set free parameters
%     motor_noise_strength = 0.005;
%     motor_noise_strength = 0;
    torque_noise_strength = 0;
    neural_noise_strength = 0;
    tendon_spring_order = 1;
    force_velocity_slope_factor = 2;
    theta_init = -0.05;

    d_l_CE = 0.06;
    d_v_CE = 0.03;
    d_F_CE = 0.03;
    d_theta = 0.06;
    d_thetaDot = 0.12;
    d_thetaTwoDot = 0.12;

    momentArms = A1;
    degrees_of_freedom = 1;
    model_ground_reaction_forces = 0;
    
    % force plate displacement
    applyDisplacement = 0;
    displacementOnsetTime = [];
    displacementTime = [];
    displacement = [];
    displacementDof = [];
    displacementType = [];
    if applyDisplacement
        model_ground_reaction_forces = 1;
        % plane inclination, Loram Lakie 2002 paradigm
%         motor_noise_strength = 0.005;
%         motor_noise_strength = 0.00;
        displacementOnsetTime = .5;
        displacementTime = 0.14;
        displacement = - deg2rad(0.055);
%         displacement = - deg2rad(0.04);
%         displacement = - deg2rad(0.0);
        displacementDof = 3;
        displacementType = 2; % 0 = ramp, 1 = sinusoid, 2 = transient sinusoid
        
        
    end
    
    if model_ground_reaction_forces
        plant = PosturePlant(degrees_of_freedom, model_ground_reaction_forces, 1.8, 80);
    else
        plant = PosturePlant_symbolic(1);
    end

    %% initialize

    % history
    l_MTC_history = zeros(1, length(time));
    l_CE_history = zeros(1, length(time));
    l_SE_history = zeros(1, length(time));
    theta_history = zeros(1, length(time));
    thetaDot_history = zeros(1, length(time));
    thetaTwoDot_history = zeros(1, length(time));
    v_CE_history = zeros(1, length(time));
    gammaRelative_history = zeros(1, length(time));
    q_history = zeros(1, length(time));
    STIM_history = zeros(1, length(time));
    lambda_history = zeros(1, length(time));
    lambdaDot_history = zeros(1, length(time));
    F_SE_history = zeros(1, length(time));
    T_M_history = zeros(1, length(time));
    T_G_history = zeros(1, length(time));
    T_damping_history = zeros(1, length(time));
    p_head_history = zeros(1, length(time));
    v_head_history = zeros(1, length(time));
    a_head_history = zeros(1, length(time));

    thetaHat_history = zeros(1, length(time));
    thetaDotHat_history = zeros(1, length(time));
    thetaTwoDotHat_history = zeros(1, length(time));
    tau_history = zeros(1, length(time));
    l_CEHat_history = zeros(1, length(time));
    v_CEHat_history = zeros(1, length(time));
    F_SEHat_history = zeros(1, length(time));
    forcePlatePosition_history = zeros(3, length(time));
    forcePlateVelocity_history = zeros(3, length(time));
    forcePlateAcceleration_history = zeros(3, length(time));
    cop_history = zeros(length(time), 1);
    fy_history = zeros(length(time), 1);
    fz_history = zeros(length(time), 1);
    mx_history = zeros(length(time), 1);
    p_head_hat_history = zeros(1, length(time));
    v_head_hat_history = zeros(1, length(time));
    a_head_hat_history = zeros(1, length(time));

    % initialize
    % theta_history(1) = -0.194392057667128;
    theta_history(1) = theta_init;
%     theta_history(1) = -0.0;
%     theta_history(1) = -0.1;
    thetaDot_history(1) = initial_velocity_perturbation;
    plant.mJointAngles(plant.mContactDofs+1:end) = theta_history(1);
    plant.mJointVelocities(plant.mContactDofs+1:end) = thetaDot_history(1);
    plant = plant.updateInternals();
    l_MTC_history(1) = A0 + A1*theta_history(1);
    T_G_history(1) = plant.mGravitationalTorques(plant.mContactDofs+1:end);
    F_G = - momentArms^(-1) * T_G_history(1);
    F_CE_rel = F_G / F_max;
    F_SE_history(1) = F_G;
    T_M_history(1) = - momentArms * F_SE_history(1);
    l_SE_history(1) = (F_G / K_SE)^(1/tendon_spring_order) + l_SE_0;
    l_CE_history(1) = l_MTC_history(1) - l_SE_history(1);
    v_CE_history(1) = 0.0;
    l_CE_rel = l_CE_history(1) / l_CE_opt;
    F_isom_n = -a * l_CE_rel.^2 + 2*a.*l_CE_rel - a + 1;
    q_history(1) = F_CE_rel / F_isom_n;
    rho = muscle_c * muscle_eta * (muscle_k - 1) ./ (muscle_k - l_CE_rel) .* l_CE_rel;
    radiant = ((muscle_q0-q_history(1)).*(q_history(1)-1).^(-1));
    STIM = rho.^(-1) .* (abs(radiant)).^(1/3) .* sign(radiant);
    STIM = min([1 STIM]);
    STIM = max([0 STIM]);
    
    STIM_0 = STIM; % CHANGED 2018-10-16, added as reference to keep STIM constant
    STIM_history(1) = STIM;
    
    gammaRelative_history(1) = STIM_history(1);
    lambda_history(1) = l_CE_history(1) - 1 / K_l * STIM_history(1);

    thetaHat_history(1) = theta_history(1);
    thetaDotHat_history(1) = thetaDot_history(1);
    thetaTwoDotHat_history(1) = thetaTwoDot_history(1);
    l_CEHat_history(1) = l_CE_history(1);
    v_CEHat_history(1) = v_CE_history(1);
    F_SEHat_history(1) = F_SE_history(1);
    F_SE_ref = F_SE_history(1);
    theta_ref = theta_history(1);

    p_head_history(1) = plant.mEndEffectorPosition(2);
    v_head_history(1) = plant.mEndEffectorVelocity(2);
    a_head_history(1) = plant.mEndEffectorAcceleration(2);
    p_head_hat_history(1) = plant.mEndEffectorPosition(2);
    v_head_hat_history(1) = plant.mEndEffectorVelocity(2);
    a_head_hat_history(1) = plant.mEndEffectorAcceleration(2);
    
    
    % perturb
    theta_history(1) = theta_history(1) + initial_angle_perturbation;
    plant.mJointAngles(plant.mContactDofs + 1 : end) = theta_history(1);
    plant = plant.updateInternals();
    
    % noise
    noise_frequency_center = 5;
    stream = RandStream('mt19937ar');
    reset(stream, random_number_seed);
%     noise_motor = lorentzNoise(stream, total_time, eulerStep, noise_frequency_center, motor_noise_strength);
%     noise_torque = lorentzNoise(stream, total_time, eulerStep, noise_frequency_center, torque_noise_strength);
%     noise_neural = lorentzNoise(stream, total_time, eulerStep, noise_frequency_center, neural_noise_strength);
    noise_motor = filteredNoise(stream, total_time, eulerStep, noise_frequency_center, motor_noise_strength);
    noise_motor = filteredNoise(stream, total_time, eulerStep, noise_frequency_center, motor_noise_strength) * 0; % CHANGED to remove noise
    noise_torque = filteredNoise(stream, total_time, eulerStep, noise_frequency_center, torque_noise_strength);
    noise_neural = filteredNoise(stream, total_time, eulerStep, noise_frequency_center, neural_noise_strength);
    % figure; axes; plot(time, noise_motor);

    %% simulate
    fallen = 0;
    if verbose; tic; end
    for i_time = 2 : length(time)

        % update sensor information
        delay_theta = d_theta * eulerStep^(-1);
        delay_thetaDot = d_thetaDot * eulerStep^(-1);
        delay_thetaTwoDot = d_thetaTwoDot * eulerStep^(-1);

        thetaHat_history(i_time) = theta_history(max(1, i_time - delay_theta));
        thetaDotHat_history(i_time) = thetaDot_history(max(1, i_time - delay_thetaDot));
        thetaTwoDotHat_history(i_time) = thetaTwoDot_history(max(1, i_time - delay_thetaTwoDot));
        
        p_head_hat_history(i_time) = p_head_history(max(1, i_time - delay_theta));
        v_head_hat_history(i_time) = v_head_history(max(1, i_time - delay_thetaDot));
        a_head_hat_history(i_time) = a_head_history(max(1, i_time - delay_thetaTwoDot));
        
        delay_l_CE = d_l_CE * eulerStep^(-1);
        l_CEHat_history(i_time) = l_CE_history(max(1, i_time - delay_l_CE));
        delay_v_CE = d_v_CE * eulerStep^(-1);
        v_CEHat_history(i_time) = v_CE_history(max(1, i_time - delay_v_CE));
        delay_F_CE = d_F_CE * eulerStep^(-1);
        F_SEHat_history(i_time) = F_SE_history(max(1, i_time - delay_F_CE));
        
        % calculate lambda dynamics
        lambda_history(i_time) = lambda_history(i_time-1) + eulerStep * lambdaDot_history(i_time-1);

        % calculate muscle stimulation
        STIM = K_l*(l_CEHat_history(i_time-1) - lambda_history(i_time-1)) ...
               + K_v*(v_CEHat_history(i_time-1) - lambdaDot_history(i_time-1)) ...
               + K_f*(F_SEHat_history(i_time-1) - F_SE_ref) ...
               + noise_motor(i_time) ...
               ;
        STIM = min([1 STIM]);
        STIM = max([0 STIM]);
%         STIM = STIM_0; % CHANGED 2018-10-16
        STIM_history(i_time) = STIM;

        % calculate muscle activation
        gammaRelativeDot = muscle_m * (STIM_history(i_time) - gammaRelative_history(i_time-1));
        l_CE_rel = l_CE_history(i_time-1) / l_CE_opt;
        rho = muscle_c * muscle_eta * (muscle_k - 1) ./ (muscle_k - l_CE_rel) .* l_CE_rel;
        q = (muscle_q0 + (rho .* gammaRelative_history(i_time-1)).^3) ./ (1 + (rho .* gammaRelative_history(i_time-1)).^3);
        q_history(i_time) = q;

        % calculate tendon force
        F_SE = K_SE * (l_SE_history(i_time-1) - l_SE_0).^tendon_spring_order;
        F_SE(l_SE_history(i_time-1) - l_SE_0 < 0) = 0;
        F_SE_history(i_time) = F_SE;
        F_CE_rel = F_SE_history(i_time) / F_max;

        % calculate muscle velocity
        F_isom_n = -a * l_CE_rel.^2 + 2*a.*l_CE_rel - a + 1;
        aStar_rel = a_rel;
        if l_CE_history(i_time-1) > l_CE_opt
            aStar_rel = a_rel * F_isom_n;
        end
        bStar_rel = b_rel;
        if q < q_crit
            bStar_rel = b_rel * (1 - 0.9*((q - q_crit).*(q_0 - q_crit)^(-1)))^2;
        end
        if F_CE_rel < q*F_isom_n
            % concentric
            v_CE_rel = bStar_rel*(F_CE_rel - q*F_isom_n) .* (F_CE_rel + q*aStar_rel).^(-1);
        else
            % eccentric
            p4 = -0.001*sign(F_isom_n);
            p1 = force_velocity_slope_factor^(-1) * 0.5*q*F_isom_n * (p4 + q*(aStar_rel+F_isom_n).*(bStar_rel)^(-1)).^(-1);
            p2 = -0.5 * q * F_isom_n * p1;
            p3 = -1.5 * q * F_isom_n;

            pq_p = 1/p4*(F_CE_rel+p3+p1*p4);
            pq_q = 1/p4*(F_CE_rel*p1+p1*p3-p2);
            v_CE_rel = -0.5*pq_p + (pq_p.^2*0.25 - pq_q).^(0.5);
        end
        v_CE = v_CE_rel * l_CE_opt;

        % calculate torques
        T_M_history(i_time) = - momentArms * F_SE;
        T_damping = - passive_damping_gain * thetaDot_history(i_time-1);
        T_G_history(i_time) = plant.mGravitationalTorques(plant.mContactDofs+1:end);
        T_damping_history(i_time) = T_damping;

        constraint_forces = calculateConstraintForces();
        tau = [zeros(plant.mContactDofs, 1); T_M_history(i_time) + T_damping];
%         plant.mExternalTorques = tau + constraint_forces + noise_torque(i_time);
%         plant.mExternalTorques = 0;

        external_impulse = 0;
        if time(i_time) > 0.1 && time(i_time) < 0.3
            external_impulse = -50;
        end

        tau_history(i_time) = tau + external_impulse + plant.mGravitationalTorques(plant.mContactDofs+1:end);
%         plant.mExternalTorques = T_M_history(i_time) + noise_torque(i_time); % CHANGED - added external impulse, removed noise
        plant.mExternalTorques = T_M_history(i_time) + external_impulse;
        
        % calculate rate of change of state variables
        thetaTwoDot = plant.mInertiaMatrix^(-1) * ...
                          (plant.mExternalTorques ...
                            - plant.mGravitationalTorques ...
                            - plant.mCoriolisMatrix * plant.mJointVelocities ...
                          ); ...
        thetaDot = plant.mJointVelocities + eulerStep * plant.mJointAccelerations;
        theta = plant.mJointAngles + eulerStep * plant.mJointVelocities;% + eulerStep^2 * plant.mJointAccelerations;
        plant.mJointAngles = theta;
        plant = plant.updateInternals();

        theta_history(i_time) = theta(plant.mContactDofs+1:end);
        thetaDot_history(i_time) = thetaDot(plant.mContactDofs+1:end);
        l_CE_history(i_time) = l_CE_history(i_time-1) + eulerStep * v_CE;
        v_CE_history(i_time) = v_CE;
        gammaRelative_history(i_time) = gammaRelative_history(i_time-1) + eulerStep * gammaRelativeDot;

        % update dependent variables
        l_MTC_history(i_time) = A0 + A1*theta_history(i_time);
        l_SE_history(i_time) = l_MTC_history(i_time) - l_CE_history(i_time);
        plant.mJointAccelerations = thetaTwoDot;
        plant.mJointVelocities = thetaDot;
        plant = plant.updateInternals();
        thetaTwoDot_history(i_time) = thetaTwoDot(plant.mContactDofs+1:end);
        if model_ground_reaction_forces
            forcePlatePosition_history(:, i_time) = theta(1:plant.mContactDofs);
            forcePlateVelocity_history(:, i_time) = thetaDot(1:plant.mContactDofs);
            forcePlateAcceleration_history(:, i_time) = thetaTwoDot(1:plant.mContactDofs);
        end
        p_head_history(i_time) = plant.mEndEffectorPosition(2);
        v_head_history(i_time) = plant.mEndEffectorVelocity(2);
        a_head_history(i_time) = plant.mEndEffectorAcceleration(2);

        if model_ground_reaction_forces
            F_y = constraint_forces(1); % horizontal force at (0, 0, 0) in anterior-posterior direction
            F_z = constraint_forces(2); % vertical ground reaction force at (0, 0, 0)
            M_x = constraint_forces(3); % torque around x-axis through (0, 0, z_s) = location of hypothetical sensor
            z_s = plant.mReferenceJointTwists{3}(2); % location of hypothetical sensor on z-axis
            % cop is calculated relative to the rotational contact DoF, apply an offset to transform to world coords
            cop_history(i_time) = (F_y*z_s + M_x) / F_z;
            
            fy_history(i_time) = F_y;
            fz_history(i_time) = F_z;
            mx_history(i_time) = M_x;
            
        end
        
        
        % apply central modulation on lambda
        lambdaDot_history(i_time) = ...
            alpha_theta * (thetaHat_history(i_time) - theta_ref) ...
            - alpha_v * v_head_hat_history(i_time) ...
            - alpha_a * a_head_hat_history(i_time) ...
            - alpha_f * (F_SEHat_history(i_time-1) - F_SE_ref) ...
            ;
%         lambdaDot_history(i_time) = 0; % CHANGED to remove higher feedback, 2018-10-17

        % check for exit criterion
        if abs(theta_history(i_time) - theta_history(1)) > 1
            fallen = 1;
        end
        if T_M_history(i_time) > ankle_torque_threshold
            fallen = 1;
        end
%         if (STIM_history(i_time) == 1) || (STIM_history(i_time) == 0)
%             fallen = 1;
%         end
        
        if fallen
            if verbose
                disp('fallen down, exiting')
            end
            break
        end
        stance_time = time(i_time);
    end
    if verbose; toc; end


    
    %% plot
    if show_plots
        
        % analyze
%         F_z = 784.6; % gravitational force acting on the body
%         sdf_max_interval = 10;
%         analysis_start_time = 5;
%         analysis_data_points = analysis_start_time * eulerStep^(-1) + 1 : total_time * eulerStep^(-1);
%         [r_length_angle, tau] = normalizedCrossCovariance(l_CE_history(analysis_data_points), theta_history(analysis_data_points), 1/eulerStep, -.5, .5);
%         cop_history = T_M_history(analysis_data_points) * F_z^(-1);
%         [sdf_data, delta_t] = stabilogramDiffusion(cop_history, eulerStep, sdf_max_interval);
    
        title_string = 'central feedback';

        % plot
        figure_width = 1200;

        figure('position', [1200 1050 figure_width 300]); axes; hold on; title('state of the muscle-tendon complex')
%         plot(time, l_MTC_history(1, :), '-', 'color', [0.5 0 1],'linewidth', 2, 'displayname', 'l_{MTC}');
%         plot(time, l_CE_history(1, :), '-', 'color', [0.5 0 1],'linewidth', 2, 'displayname', 'l_{CE}');
%         plot(time, l_CEHat_history(1, :), '--', 'color', [0.5 0 1],'linewidth', 2, 'displayname', 'l_{CE}^{hat}');
        plot(time, lambda_history(1, :), 'g-', 'linewidth', 1, 'displayname', '\lambda');
        plot(time, lambdaDot_history(1, :), 'g--', 'linewidth', 1, 'displayname', 'd \lambda / dt');
%         plot(time, v_CE_history(1, :), '-', 'color', [0.5 1 0],'linewidth', 2, 'displayname', 'v_{CE}');
%         plot(time, v_CEHat_history(1, :), '--', 'color', [0.5 1 0],'linewidth', 2, 'displayname', 'v_{CE}^{hat}');
%         plot(time, l_SE_history(1, :), 'r-', 'linewidth', 2, 'displayname', 'l_{SE}');
        plot(time, STIM_history(1, :), 'c-', 'linewidth', 2, 'displayname', 'STIM');
%         plot(time, q_history(1, :), 'b--', 'linewidth', 1, 'displayname', 'q');
%         plot(time, gammaRelative_history(1, :), 'r-', 'linewidth', 1, 'displayname', '\gamma_{rel}');
%         plot(time, F_SE_history(1, :), '-', 'color', [1 0.5 0], 'linewidth', 2, 'displayname', 'F_{SE}');
%         plot(time, F_SE_history(1, :)/F_max(1), '-', 'color', [1 0.5 0], 'linewidth', 2, 'displayname', 'F_{SE,rel}');
        % plot(time, F_CEHat_history(1, :)/F_max(1), '--', 'color', [1 0.5 0], 'linewidth', 2, 'displayname', 'F_{CE,rel}^Hat');
        legend('toggle', 'Position', 'NW')

% return
        
        figure('position', [1200 700 figure_width 300]); axes; hold on; title('joint kinematics')
%         plot(time, thetaTwoDot_history, '-.', 'color', [0 0 1],'linewidth', 2, 'displayname', 'd^2 \theta / dt^2');
%         plot(time, thetaDot_history, '--', 'color', [0 1 0],'linewidth', 2, 'displayname', 'd \theta / dt');
        plot(time, theta_history, '-', 'color', [1 0 0], 'linewidth', 2, 'displayname', '\theta');
%         plot(time, thetaHat_history, '--', 'color', [1 0 0], 'linewidth', 2, 'displayname', '\hat \theta');
        legend('toggle', 'Position', 'NW')
%         return
%         figure('position', [1200 1050 figure_width 300]); axes; hold on; title('head kinematics')
%         plot(time, p_head_history, '-', 'color', [0 0 1], 'linewidth', 2, 'displayname', 'p');
%         plot(time, v_head_history, '--', 'color', [0 0 1],'linewidth', 2, 'displayname', 'v');
%         plot(time, a_head_history, '-.', 'color', [0 0 1],'linewidth', 2, 'displayname', 'a');
%         legend('toggle', 'Position', 'NW')

return

        figure('position', [1200 350 figure_width 300]); axes; hold on; title('torques')
        title([title_string ' - torque variables'])
        plot(time, T_M_history, 'b-', 'linewidth', 2, 'displayname', 'T_M');
        plot(time, T_G_history, 'r-', 'linewidth', 2, 'displayname', 'T_G');
        legend('toggle', 'Position', 'NW')
return
%         figure('position', [1200 350 figure_width 300]); axes; hold on; title('torques')
%         title([title_string ' - center of pressure'])
%         plot(time(2:end), cop_history(2:end), 'm-', 'linewidth', 2, 'displayname', 'CoP');
%         plot(time(2:end), fy_history(2:end), 'g-', 'linewidth', 2, 'displayname', 'CoP');
%         plot(time(2:end), fz_history(2:end), 'b-', 'linewidth', 2, 'displayname', 'CoP');
%         plot(time(2:end), mx_history(2:end), 'r-', 'linewidth', 2, 'displayname', 'CoP');
        
        figure('position', [1200 0 figure_width 300]); axes; hold on;
        title([title_string ' - relative variables'])
        theta_n = (theta_history - min(theta_history)) * 1 / (max(theta_history) - min(theta_history));
        l_CE_n = (l_CE_history - min(l_CE_history)) * 1 / (max(l_CE_history) - min(l_CE_history));
        l_MTC_n = (l_MTC_history - min(l_MTC_history)) * 1 / (max(l_MTC_history) - min(l_MTC_history));
        l_SE_n = (l_SE_history - min(l_SE_history)) * 1 / (max(l_SE_history) - min(l_SE_history));
        
        plot(time, theta_n, 'b-', 'linewidth', 2, 'displayname', '\theta_n');
        plot(time, l_CE_n, 'r-', 'linewidth', 2, 'displayname', 'l_{CE, n}');
%         plot(time, l_CE_n_filtered, 'r-', 'linewidth', 2, 'displayname', 'l_{CE, n}, filtered');
%         plot(time, l_MTC_n, 'g-', 'linewidth', 2, 'displayname', 'l_{MTC, n}');
%         plot(time, l_SE_n, 'm-', 'linewidth', 2, 'displayname', 'l_{SE, n}');
        legend('show', 'Location', 'NW')

        distFig
        
        figure; axes; hold on; title('time-delayed cross correlation');
        plot(tau, r_length_angle');

        figure; axes; hold on; title('SDF');
        plot(delta_t, sdf_data', 'linewidth', 1)
        
        
        
        
%         figure; axes; hold on;
%         title([title_string ' - muscle length'])
%         plot(time, l_CE_history(1, :) * 1e03, '-', 'color', [0.25 0 1],'linewidth', 2, 'displayname', 'l_{CE}');
%         plot(time, l_CE_history_filtered(1, :) * 1e03, '-', 'color', [0.5 0 1],'linewidth', 1, 'displayname', 'l_{CE}, filtered');
%         ylabel('l_CE (mm)')
%         legend('toggle', 'Position', 'NW')

%         figure; axes; hold on;
%         title([title_string ' - motor noise'])
%         plot(time, noise_motor(1, :), '-', 'color', [0 1 1],'linewidth', 2, 'displayname', '\eta');
%         
%         figure; axes; hold on;
%         title([title_string ' - force platform'])
%         plot(time, forcePlatePosition_history(3, :)*180/pi, '-', 'color', [1 0 0],'linewidth', 2, 'displayname', 'force plate, rotation (pos | deg)');
%         plot(time, forcePlateVelocity_history(3, :)*180/pi, '-', 'color', [0 1 0],'linewidth', 2, 'displayname', 'force plate, rotation (vel | deg/sec)');
%         plot(time, forcePlateAcceleration_history(3, :)*180/pi, '-', 'color', [0 0 1],'linewidth', 2, 'displayname', 'force plate, rotation (acc | deg/sec^2)');
%         legend('toggle')
%         
%         figure; axes; hold on;
%         title([title_string ' - force platform'])
%         plot(time, forcePlatePosition_history(3, :), '-', 'color', [1 0 0],'linewidth', 2, 'displayname', 'force plate, rotation (pos | rad)');
%         plot(time, forcePlateVelocity_history(3, :), '-', 'color', [0 1 0],'linewidth', 2, 'displayname', 'force plate, rotation (vel | rad/sec)');
%         plot(time, forcePlateAcceleration_history(3, :), '-', 'color', [0 0 1],'linewidth', 2, 'displayname', 'force plate, rotation (acc | rad/sec^2)');
%         legend('toggle')

%         figure; axes; hold on;
%         title([title_string ' - head position'])
%         plot(time, headPositionTwoDotHat_history, '-', 'color', [1 0 0],'linewidth', 2, 'displayname', 'head position (acc | m/sec^2)');
%         plot(time, thetaTwoDotHat_history, '-.', 'color', [0 0 1],'linewidth', 2, 'displayname', 'd^2 \theta / dt^2');
%         plot(time, -headPositionTwoDotHat_history * 1/plant.mLinkLengths(1), '--', 'color', [0 1 0],'linewidth', 2, 'displayname', 'head position (acc | m/sec^2)');
%         legend('toggle')
    end
    
    
    
    %% constraints

    function constraintForces = calculateConstraintForces()
        if ~model_ground_reaction_forces
            constraintForces = zeros(plant.mNumberOfJoints, 1);
            return;
        end


        % calculate accelerations from force platform
        accelerationFromForcePlatform = zeros(plant.mContactDofs, 1);
        if applyDisplacement  % vertical force plate perturbation force
            t = i_time * eulerStep;
            if displacementOnsetTime <= t && t <= displacementOnsetTime+displacementTime
                if displacementType == 0
                    acceleration = rampDisplacementAcceleration(t-displacementOnsetTime, displacementTime, displacement);
                elseif displacementType == 2
                    acceleration = sinusoidDisplacementAcceleration(t-displacementOnsetTime, displacementTime, displacement);
                else
                    acceleration = 0;
                end
                accelerationFromForcePlatform(displacementDof) = -acceleration;
            end
        end

        % calculate case where knee joint limit constraint is not active
        tau = T_M_history(i_time);

        theta = plant.mJointAngles(plant.mContactDofs+1:end);
        thetaDot = plant.mJointVelocities(plant.mContactDofs+1:end);
        M_thetaGamma = plant.mInertiaMatrix(plant.mContactDofs+1:end, 1:plant.mContactDofs);
        M_theta = plant.mInertiaMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
        C_thetaGamma = plant.mCoriolisMatrix(plant.mContactDofs+1:end, 1:plant.mContactDofs);
        C_theta = plant.mCoriolisMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
        N_theta = plant.mGravitationalTorques(plant.mContactDofs+1:end);
        gammaDot = plant.mJointVelocities(1:plant.mContactDofs, 1);
        gammaTwoDot = accelerationFromForcePlatform;
        thetaTwoDot_unc = M_theta^(-1) ...
                              * (...
                                  tau  ...
                                  - M_thetaGamma * gammaTwoDot ...
                                  - C_thetaGamma * gammaDot ...
                                  - C_theta * plant.mJointVelocities(plant.mContactDofs+1:end) ...
                                  - N_theta ...
                               );

        M_gamma = plant.mInertiaMatrix(1:plant.mContactDofs, 1:plant.mContactDofs);
        M_gammaTheta = plant.mInertiaMatrix(1:plant.mContactDofs, plant.mContactDofs+1:end);
        C_gamma = plant.mCoriolisMatrix(1:plant.mContactDofs, 1:plant.mContactDofs);
        C_gammaTheta = plant.mCoriolisMatrix(1:plant.mContactDofs, plant.mContactDofs+1:end);
        N_gamma = plant.mGravitationalTorques(1:plant.mContactDofs, 1);
        rho = M_gamma * gammaTwoDot ...
                  + M_gammaTheta * thetaTwoDot_unc ...
                  + C_gamma * gammaDot ...
                  + C_gammaTheta * thetaDot ...
                  + N_gamma;
        constraintForces = [rho; zeros(size(tau))];




    end    
    
end









