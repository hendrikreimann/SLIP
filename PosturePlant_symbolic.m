% class for the display of a stick figure

% the stick figure has real joints and hidden joints, called contact- and
% body degrees of freedom

classdef PosturePlant_symbolic < handle
    properties
        % reference data
        mt5_ref = [0 .1075 0]';
        ankle_ref = [0 0 .1252]';
        knee_ref = [0 0 .5832]';
        hip_ref = [0 0 1.0682]';
        shoulder_ref = [0 0 1.0682]';
        head_ref = [0 0 1.9882]';
        mNumberOfJoints;
        mContactDofs = 0;
        mBodyDofs;
        mReferenceJointTwists;
        mReferenceJointTransformations;
        mReferenceLinkTransformations;
        mTransformedLinkInertiaMatrices;
        mLinkMasses;
        mLinkLengths;
        mLinkMomentsOfInertia;
        mLinkComDistancesFromJoint;
        mPassiveStabilityStiffnessThresholds;
        
        % current state - externals
        mJointAngles;
        mJointVelocities;
        mJointAccelerations;
        mExternalTorques;
        jointPositions;
        
        mInertiaMatrix;
        mCoriolisMatrix;
        mGravitationalTorques;
        mEndEffectorJacobian;
        mComJacobian;

        
        % functional variables
        mEndEffectorPosition;
        mEndEffectorVelocity;
        mEndEffectorAcceleration;
        mEndEffectorOrientation;
        mCom;
        mComVelocity;
        gravitationJacobian;
        gravitationSummedSquaresJacobian;
        
    end
    methods
        function obj = PosturePlant_symbolic(bodyDofs, height, mass)
            if nargin < 3
                mass = 80;
            end
            if nargin < 2
                height = 1.8;
            end
            obj.mBodyDofs = bodyDofs;
            obj.mNumberOfJoints = obj.mBodyDofs;
            obj.mLinkMasses = zeros(obj.mNumberOfJoints, 1);
            obj.mLinkLengths = zeros(obj.mNumberOfJoints, 1);
            obj.mLinkMomentsOfInertia = ones( obj.mBodyDofs, 3 );


            % anthropometric measures
            if (obj.mBodyDofs == 1) 
                
                % segment center of mass
                com = 1;

                body_inertia = 27;

                % save into lists
                obj.mLinkMasses = mass;
                obj.mLinkLengths = height;
                obj.mLinkMomentsOfInertia( 1, 1 ) = body_inertia;
                obj.mLinkComDistancesFromJoint(1) = com;
                
                
                
                
                
                
                
            end
            if (obj.mBodyDofs == 2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% new version, from Winter's book
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % segment geometry
                obj.ankle_ref = [0; 0; 0.039*height];
                obj.knee_ref = [0; 0; 0.285*height];
                obj.hip_ref = [0; 0; 0.530*height];
                obj.shoulder_ref = [0; 0; 0.818*height];
                obj.head_ref = [0; 0; 0.936*height];
                shank = obj.knee_ref - obj.ankle_ref;
                thigh = obj.hip_ref - obj.knee_ref;
                GTGH = obj.shoulder_ref - obj.hip_ref; % greater trochanter to glenohumeral joint

%                 determine segment mass
                shank_mass = 0.0465*2*mass;
                thigh_mass = 0.1*2*mass;
                leg_mass = shank_mass + thigh_mass;
                trunk_mass = 0.678*mass;
                
                shank_com = obj.ankle_ref + 0.567*shank;
                thigh_com = obj.knee_ref + 0.567 * thigh;
                leg_com = (shank_mass*shank_com + thigh_mass*thigh_com) / leg_mass;
                trunk_com = obj.hip_ref + 0.626 * GTGH;
                
                % segment radius of gyration from center of mass, or other joints if specified
                shank_rog = 0.302 * norm(shank);
                thigh_rog = 0.323 * norm(thigh);
                trunk_rog = 0.496 * norm(GTGH);
                
                % segment moments of inertia around the x-axis
                shank_inertia = shank_mass * shank_rog^2;
                thigh_inertia = thigh_mass * thigh_rog^2;
                shank_inertia_leg = shank_inertia + shank_mass * (norm(leg_com-shank_com))^2; % parallel axis theorem
                thigh_inertia_leg = thigh_inertia + thigh_mass * (norm(leg_com-thigh_com))^2; % parallel axis theorem
                leg_inertia = shank_inertia_leg + thigh_inertia_leg;
                trunk_inertia = trunk_mass * trunk_rog^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% formatting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % save into lists
                obj.mLinkMasses = [leg_mass, trunk_mass];
                obj.mLinkLengths = [norm(obj.hip_ref - obj.ankle_ref), norm(obj.head_ref - obj.hip_ref)];
                obj.mLinkMomentsOfInertia( 1, 2 ) = leg_inertia;
                obj.mLinkMomentsOfInertia( 2, 2 ) = trunk_inertia;
                obj.mLinkComDistancesFromJoint(1) = norm(leg_com - obj.ankle_ref);
                obj.mLinkComDistancesFromJoint(2) = norm(trunk_com - obj.hip_ref);
            end
            if (obj.mBodyDofs == 3)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% new version, from Winter's book
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % segment geometry
                obj.ankle_ref = [0; 0; 0.039*height];
                obj.knee_ref = [0; 0; 0.285*height];
                obj.hip_ref = [0; 0; 0.530*height];
                obj.shoulder_ref = [0; 0; 0.818*height];
                obj.head_ref = [0; 0; 0.936*height];
                shank = obj.knee_ref - obj.ankle_ref;
                thigh = obj.hip_ref - obj.knee_ref;
                GTGH = obj.shoulder_ref - obj.hip_ref; % greater trochanter to glenohumeral joint

%                 determine segment mass
                shank_mass = 0.0465*2*mass;
                thigh_mass = 0.1*2*mass;
                trunk_mass = 0.678*mass;
                
                shank_com = obj.ankle_ref + 0.567*shank;
                thigh_com = obj.knee_ref + 0.567 * thigh;
                trunk_com = obj.hip_ref + 0.626 * GTGH;
                
                % segment radius of gyration from center of mass, or other joints if specified
                shank_rog = 0.302 * norm(shank);
                thigh_rog = 0.323 * norm(thigh);
                trunk_rog = 0.496 * norm(GTGH);
                
                % segment moments of inertia around the x-axis
                shank_inertia = shank_mass * shank_rog^2;
                thigh_inertia = thigh_mass * thigh_rog^2;
                trunk_inertia = trunk_mass * trunk_rog^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% formatting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % save into lists
                obj.mLinkMasses = [shank_mass, thigh_mass, trunk_mass];
                obj.mLinkLengths = [norm(obj.knee_ref - obj.ankle_ref), norm(obj.hip_ref - obj.knee_ref), norm(obj.head_ref - obj.hip_ref)];
                obj.mLinkMomentsOfInertia( 1, 1 ) = shank_inertia;
                obj.mLinkMomentsOfInertia( 2, 1 ) = thigh_inertia;
                obj.mLinkMomentsOfInertia( 3, 1 ) = trunk_inertia;
                obj.mLinkComDistancesFromJoint(1) = norm(shank_com - obj.ankle_ref);
                obj.mLinkComDistancesFromJoint(2) = norm(thigh_com - obj.knee_ref);
                obj.mLinkComDistancesFromJoint(3) = norm(trunk_com - obj.hip_ref);
            end
            obj.mJointAngles = zeros(obj.mNumberOfJoints, 1);
            obj.mJointVelocities = zeros(obj.mNumberOfJoints, 1);
            obj.mJointAccelerations = zeros(obj.mNumberOfJoints, 1);
            obj.mExternalTorques = zeros(obj.mNumberOfJoints, 1);
            obj.jointPositions = zeros(3, obj.mNumberOfJoints+1);
            obj = obj.updateInternals();
            obj.mPassiveStabilityStiffnessThresholds = obj.calculatePassiveStiffnessThresholds();
            obj.mEndEffectorOrientation = sum(obj.mJointAngles);
            obj.gravitationJacobian = zeros(obj.mBodyDofs);

            
        end % constructor
        function thresholds = calculatePassiveStiffnessThresholds(obj)
            % calculates the minimum stiffnes value for passive stability for each joint, assuming all other joints are
            % fixed at 0
            g = 9.81;
            thresholds = zeros(obj.mBodyDofs, 1);
            
            h(1) = obj.ankle_ref(3) + cos(obj.mJointAngles(1))*obj.mLinkComDistancesFromJoint(1);
            if obj.mBodyDofs > 1
                h(2) = obj.ankle_ref(3) + cos(obj.mJointAngles(1))*obj.mLinkLengths(1) + cos(obj.mJointAngles(1) + obj.mJointAngles(2))*obj.mLinkComDistancesFromJoint(2);
            end
            if obj.mBodyDofs > 2
                h(3) = obj.ankle_ref(3) + cos(obj.mJointAngles(1))*obj.mLinkLengths(1) + cos(obj.mJointAngles(1) + obj.mJointAngles(2))*obj.mLinkLengths(2) + cos(obj.mJointAngles(1) + obj.mJointAngles(2) + obj.mJointAngles(3))*obj.mLinkComDistancesFromJoint(3);            
            end
            for j = 1 : obj.mBodyDofs
                for k = j : obj.mBodyDofs
                    thresholds(j-obj.mContactDofs) = thresholds(j-obj.mContactDofs) + obj.mLinkMasses(k)*h(k);
                end
            end
            thresholds = thresholds*g;
        end
        function damping = calculateCriticalDampingParameters(obj, stiffness)
            % calculates the damping parameters for critical damping at a given stiffness value
            % seperately for each joint, assuming the other joints are fixed
            damping = zeros(obj.mBodyDofs, 1);
%             for j = 1 : obj.mBodyDofs
%                 % inertia of all links further up the chain around the j-th joint axes
%                 I = 0;
%                 h = obj.mReferenceJointTransformations{j}(3, 4); % height of the j-th joint axis
%                 for k = j : obj.mBodyDofs
%                     r = obj.mReferenceLinkTransformations{k}(3, 4) - h; % distance of k-th link segment CoM from the j-th joint angle
%                     I = I + r^2*obj.mLinkMasses(k);
%                 end
%                 % undamped angular frequency
%                 omega = sqrt(stiffness(j-obj.mContactDofs) / I);
%                 damping(j) = 2*I*omega;
%             end
            
            
        end
        function obj = updateInternals(obj)
            if obj.mBodyDofs == 1
                m1 = obj.mLinkMasses(1);
                I1 = obj.mLinkMomentsOfInertia(1);
                r1 = obj.mLinkComDistancesFromJoint(1);
                l1 = obj.mLinkLengths(1);

                c1 = cos(obj.mJointAngles(1));
                s1 = sin(obj.mJointAngles(1));
                td1 = obj.mJointVelocities(1);
                tau1 = td1;
                t2d1 = obj.mJointAccelerations(1);
                rho1 = t2d1;

                M11 = I1 + m1*r1^2;

                C11 = 0;

                g = 9.81;
                N1 = - (g*m1*r1*s1);

                p1 = 0;
                p2 = - s1*l1;
                p3 = obj.ankle_ref(3) + c1*l1;
                obj.jointPositions = repmat(obj.ankle_ref, 1, 2) + [0, 0; ...
                                                                    0, -s1*l1; ...
                                                                    0, c1*l1];

                J11 = 0;
                J21 = - l1*c1;
                J31 = - l1*s1;
                
                CoM1 = r1 * [0; -s1; c1];

                J_M21 = (- r1*c1);
                J_M31 = (- r1*s1);
                

                obj.mInertiaMatrix = M11;
                obj.mCoriolisMatrix = C11;
                obj.mGravitationalTorques = N1;
                obj.mEndEffectorJacobian = [J11; J21; J31];
                obj.mCom = obj.ankle_ref + CoM1;
                obj.mComJacobian = [0; J_M21; J_M31];

                % update functionals
                obj.mEndEffectorPosition = [p1; p2; p3];
                obj.mEndEffectorVelocity = obj.mEndEffectorJacobian * obj.mJointVelocities;
                obj.mEndEffectorAcceleration = [0 0 0]';
                obj.mEndEffectorOrientation = sum(obj.mJointAngles);
                obj.mComVelocity = obj.mComJacobian * obj.mJointVelocities;
                
                obj.mEndEffectorAcceleration = ...
                [...
                 0; ...
                 - l1*(rho1*c1-tau1^2*s1); ...
                 - l1*(rho1*s1+tau1^2*c1); ...
                ];
                
                obj.gravitationJacobian(1, 1) = - (g*m1*r1*c1);
                
                dG_by_dN = 2*N1;
                obj.gravitationSummedSquaresJacobian = dG_by_dN * obj.gravitationJacobian;
            elseif obj.mBodyDofs == 2
                m1 = obj.mLinkMasses(1);
                m2 = obj.mLinkMasses(2);
                I1 = obj.mLinkMomentsOfInertia(1, 1);
                I2 = obj.mLinkMomentsOfInertia(2, 1);
                r1 = obj.mLinkComDistancesFromJoint(1);
                r2 = obj.mLinkComDistancesFromJoint(2);
                l1 = obj.mLinkLengths(1);
                l2 = obj.mLinkLengths(2);

                c1 = cos(obj.mJointAngles(1));
                c2 = cos(obj.mJointAngles(2));
                c12 = cos(obj.mJointAngles(1) + obj.mJointAngles(2));
                s1 = sin(obj.mJointAngles(1));
                s2 = sin(obj.mJointAngles(2));
                s12 = sin(obj.mJointAngles(1) + obj.mJointAngles(2));
                td1 = obj.mJointVelocities(1);
                td2 = obj.mJointVelocities(2);
                t2d1 = obj.mJointAccelerations(1);
                t2d2 = obj.mJointAccelerations(2);
                
                tau1 = td1;
                tau2 = td1+td2;
                rho1 = t2d1;
                rho2 = t2d1+t2d2;


                M11 = I1 + I2 + m1*r1^2 + m2*(l1^2 + 2*l1*r2*c2 + r2^2);
                M12 = I2 + m2*(r2^2 + l1*r2*c2);
                M22 = I2 + m2*r2^2;

                C11 = -td2*m2*l1*r2*s2;
                C12 = -(td2 + td1) * m2*l1*r2*s2;

                C21 = td1*(l1*m2*r2*s2);
                C22 = 0;

                g = 9.81;
                N1 = - (g*m2*(r2*s12 + l1*s1)) - (g*m1*r1*s1);
                N2 = - (g*m2*r2*s12);

                p1 = 0;
                p2 = - s1*l1 - s12*l2;
                p3 = obj.ankle_ref(3) + c1*l1 + c12*l2;
                obj.jointPositions = repmat(obj.ankle_ref, 1, 3) + [0, 0, 0; ...
                                                                    0, -s1*l1, -s1*l1-s12*l2; ...
                                                                    0, c1*l1, c1*l1+c12*l2];

                J11 = 0;
                J12 = 0;
                J21 = - l2*c12 - l1*c1;
                J22 = - l2*c12;
                J31 = - l2*s12 - l1*s1;
                J32 = - l2*s12;
                
                CoM1 = r1 * [0; -s1; c1];
                CoM2 = l1*[0; -s1; c1] + r2*[0; -s12; c12];

                J_M21 = (- m1*r1*c1 - m2*(l1*c1 + r2*c12)) / (m1+m2);
                J_M22 = (- m2*r2*c12) / (m1+m2);
                J_M31 = (- m1*r1*s1 - m2*(l1*s1 + r2*s12)) / (m1+m2);
                J_M32 = (- m2*r2*s12) / (m1+m2);
                

                obj.mInertiaMatrix = [M11 M12; M12 M22];
                obj.mCoriolisMatrix = [C11 C12; C21 C22];
                obj.mGravitationalTorques = [N1; N2];
                obj.mEndEffectorJacobian = [J11 J12; J21 J22; J31 J32];
                obj.mCom = obj.ankle_ref + (m1*CoM1 + m2*CoM2) / sum(obj.mLinkMasses);
                obj.mComJacobian = [zeros(1, 2); J_M21 J_M22; J_M31 J_M32];

                % update functionals
                obj.mEndEffectorPosition = [p1; p2; p3];
                obj.mEndEffectorVelocity = obj.mEndEffectorJacobian * obj.mJointVelocities;
                obj.mEndEffectorAcceleration = ...
                [...
                 0; ...
                 - l1*(rho1*c1-tau1^2*s1) - l2*(rho2*c12-tau2^2*s12); ...
                 - l1*(rho1*s1+tau1^2*c1) - l2*(rho2*s12+tau2^2*c12); ...
                ];
                obj.mEndEffectorOrientation = sum(obj.mJointAngles);
                obj.mComVelocity = obj.mComJacobian * obj.mJointVelocities;
                
                
                obj.gravitationJacobian(1, 1) = - (g*m2*(r2*c12 + l1*c1)) - (g*m1*r1*c1);
                obj.gravitationJacobian(1, 2) = - g*m2*r2*c12;
                obj.gravitationJacobian(2, 1) = - g*m2*r2*c12;
                obj.gravitationJacobian(2, 2) = - g*m2*r2*c12;
                
                dG_by_dN = [2*N1 2*N2];
                obj.gravitationSummedSquaresJacobian = dG_by_dN * obj.gravitationJacobian;
                

            elseif obj.mBodyDofs == 3
                m1 = obj.mLinkMasses(1);
                m2 = obj.mLinkMasses(2);
                m3 = obj.mLinkMasses(3);
                I1 = obj.mLinkMomentsOfInertia(1, 1);
                I2 = obj.mLinkMomentsOfInertia(2, 1);
                I3 = obj.mLinkMomentsOfInertia(3, 1);
                r1 = obj.mLinkComDistancesFromJoint(1);
                r2 = obj.mLinkComDistancesFromJoint(2);
                r3 = obj.mLinkComDistancesFromJoint(3);
                l1 = obj.mLinkLengths(1);
                l2 = obj.mLinkLengths(2);
                l3 = obj.mLinkLengths(3);

                c1 = cos(obj.mJointAngles(1));
                c2 = cos(obj.mJointAngles(2));
                c3 = cos(obj.mJointAngles(3));
                c12 = cos(obj.mJointAngles(1) + obj.mJointAngles(2));
                c23 = cos(obj.mJointAngles(2) + obj.mJointAngles(3));
                c123 = cos(obj.mJointAngles(1) + obj.mJointAngles(2) + obj.mJointAngles(3));
                s1 = sin(obj.mJointAngles(1));
                s2 = sin(obj.mJointAngles(2));
                s3 = sin(obj.mJointAngles(3));
                s12 = sin(obj.mJointAngles(1) + obj.mJointAngles(2));
                s23 = sin(obj.mJointAngles(2) + obj.mJointAngles(3));
                s123 = sin(obj.mJointAngles(1) + obj.mJointAngles(2) + obj.mJointAngles(3));
                td1 = obj.mJointVelocities(1);
                td2 = obj.mJointVelocities(2);
                td3 = obj.mJointVelocities(3);
                t2d1 = obj.mJointAccelerations(1);
                t2d2 = obj.mJointAccelerations(2);
                t2d3 = obj.mJointAccelerations(3);
                tau1 = td1;
                tau2 = td1+td2;
                tau3 = td1+td2+td3;
                rho1 = t2d1;
                rho2 = t2d1+t2d2;
                rho3 = t2d1+t2d2+t2d3;

                M11 = I1 + I2 + I3 + m1*r1^2 + m2*(l1^2 + 2*l1*r2*c2 + r2^2) + m3*l1^2 + m3*l2^2 + m3*r3^2 + m3*(2*l1*l2*c2 + 2*l1*r3*c23 + 2*l2*r3*c3);
                M12 = I2 + I3 + m2*(r2^2 + l1*r2*c2) + m3*(l2^2 + r3^2 + l1*l2*c2 + l1*r3*c23 + 2*l2*r3*c3);
                M13 = I3 + m3*(r3^2 + l1*r3*c23 + l2*r3*c3);
                M22 = I2 + I3 + m2*r2^2 + m3*(l2^2 + 2*l2*r3*c3 + r3^2);
                M23 = I3 + m3*(r3^2 + l2*r3*c3);
                M33 = I3 + m3*r3^2;

                C11 = td3*(-m3*l1*r3*s23 - m3*l2*r3*s3) + td2*((-m2*l1*r2-m3*l1*l2)*s2 - m3*l1*r3*s23);
                C12 = (td3)*(-m3*l1*r3*s23 - m3*l2*r3*s3) + (td2 + td1) * ((-m2*l1*r2-m3*l1*l2)*s2 - m3*l1*r3*s23);
                C13 = (td1+td2+td3) * (-m3*l1*r3*s23 - m3*l2*r3*s3);

                C21 = td1*((m3*(l1*l2*s2 + l1*r3*s23)) + l1*m2*r2*s2) - td3*((m3*(2*l2*r3*s3 + l1*r3*s23))/2 - (l1*m3*r3*s23)/2);
                C22 = -l2*m3*r3*td3*s3;
                C23 = - td1*((m3*(2*l2*r3*s3 + l1*r3*s23))/2 - (l1*m3*r3*s23)/2) - l2*m3*r3*td2*s3 - l2*m3*r3*td3*s3;

                C31 = (td1 + td2) * (m3*l2*r3*s3) + td1 * (m3*l1*r3*s23);
                C32 = (td2 + td1) * (m3*l2*r3*s3);
                C33 = 0;

                g = 9.81;
                N1 = - (g*m3*(r3*s123 + l2*s12 + l1*s1)) - (g*m2*(r2*s12 + l1*s1)) - (g*m1*r1*s1);
                N2 = - (g*m3*(r3*s123 + l2*s12)) - (g*m2*r2*s12);
                N3 = - (g*m3*r3*s123);

                p1 = 0;
                p2 = - s1*l1 - s12*l2 - s123*l3;
                p3 = obj.ankle_ref(3) + c1*l1 + c12*l2 + c123*l3;
                obj.jointPositions = repmat(obj.ankle_ref, 1, 4) + [0, 0, 0, 0; ...
                                                                    0, -s1*l1, -s1*l1-s12*l2, -s1*l1-s12*l2-s123*l3; ...
                                                                    0, c1*l1, c1*l1+c12*l2, c1*l1+c12*l2+c123*l3];

                J11 = 0;
                J12 = 0;
                J13 = 0;
                J21 = - l2*c12 - l1*c1 - l3*c123;
                J22 = - l2*c12 - l3*c123;
                J23 = -l3*c123;
                J31 = - l2*s12 - l1*s1 - l3*s123;
                J32 = - l2*s12 - l3*s123;
                J33 = -l3*s123;
                
                CoM1 = r1 * [0; -s1; c1];
                CoM2 = l1*[0; -s1; c1] + r2*[0; -s12; c12];
                CoM3 = l1*[0; -s1; c1] + l2*[0; -s12; c12] + r3*[0; -s123; c123];

                J_M21 = (- m1*r1*c1 - m2*(l1*c1 + r2*c12) - m3*(l1*c1 + l2*c12 + r3*c123) ) / (m1+m2+m3);
                J_M22 = (- m2*r2*c12 - m3*(l2*c12 + r3*c123) ) / (m1+m2+m3);
                J_M23 = - m3*r3*c123 / (m1+m2+m3);
                J_M31 = (- m1*r1*s1 - m2*(l1*s1 + r2*s12) - m3*(l1*s1 + l2*s12 + r3*s123) ) / (m1+m2+m3);
                J_M32 = (- m2*r2*s12 - m3*(l2*s12 + r3*s123) ) / (m1+m2+m3);
                J_M33 = - m3*r3*s123 / (m1+m2+m3);

                obj.mInertiaMatrix = [M11 M12 M13; M12 M22 M23; M13 M23 M33];
                obj.mCoriolisMatrix = [C11 C12 C13; C21 C22 C23; C31 C32 C33];
                obj.mGravitationalTorques = [N1; N2; N3];
                obj.mEndEffectorJacobian = [J11 J12 J13; J21 J22 J23; J31 J32 J33];
                obj.mCom = obj.ankle_ref + (m1*CoM1 + m2*CoM2 + m3*CoM3) / sum(obj.mLinkMasses);
                obj.mComJacobian = [zeros(1, 3); J_M21 J_M22 J_M23; J_M31 J_M32 J_M33];

                % update functionals
                obj.mEndEffectorPosition = [p1; p2; p3];
                obj.mEndEffectorVelocity = obj.mEndEffectorJacobian * obj.mJointVelocities;
                obj.mEndEffectorAcceleration = [0 0 0]';

                obj.mEndEffectorVelocity = ...
                [...
                 0; ...
                 - c1*l1*tau1 - c12*l2*tau2 - c123*l3*tau3; ...
                 - s1*l1*tau1 - s12*l2*tau2 - s123*l3*tau3; ...
                ];

                obj.mEndEffectorAcceleration = ...
                [...
                 0; ...
                 - l1*(rho1*c1-tau1^2*s1) - l2*(rho2*c12-tau2^2*s12) - l3*(rho3*c123-tau3^2*s123); ...
                 - l1*(rho1*s1+tau1^2*c1) - l2*(rho2*s12+tau2^2*c12) - l3*(rho3*s123+tau3^2*c123); ...
                ];
                
                
                obj.mEndEffectorOrientation = sum(obj.mJointAngles);
                obj.mComVelocity = obj.mComJacobian * obj.mJointVelocities;
                
                
                obj.gravitationJacobian(1, 1) = - (g*m3*(r3*c123 + l2*c12 + l1*c1)) - (g*m2*(r2*c12 + l1*c1)) - (g*m1*r1*c1);
                obj.gravitationJacobian(1, 2) = - g*m3*(r3*c123 + l2*c12) - g*m2*r2*c12;
                obj.gravitationJacobian(1, 3) = - g*m3*(r3*c123);
                obj.gravitationJacobian(2, 1) = - (g*m3*(r3*c123 + l2*c12)) - (g*m2*r2*c12);
                obj.gravitationJacobian(2, 2) = - (g*m3*(r3*c123 + l2*c12)) - (g*m2*r2*c12);
                obj.gravitationJacobian(2, 3) = - g*m3*r3*c123;
                obj.gravitationJacobian(3, 1) = - (g*m3*r3*c123);
                obj.gravitationJacobian(3, 2) = - (g*m3*r3*c123);
                obj.gravitationJacobian(3, 3) = - (g*m3*r3*c123);
                
                dG_by_dN = [2*N1 2*N2 2*N3];
                obj.gravitationSummedSquaresJacobian = dG_by_dN * obj.gravitationJacobian;
                

            end
        end % function update
    end
    
end










