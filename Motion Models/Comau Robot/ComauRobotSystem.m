classdef ComauRobotSystem < matlab.System
    
    % Calculates the forward or inverse kinematics of the Comau Robot
    % Calculates the motion of the Comau robot tool-point relative to the
    % neutral coordinate system of the Stewart Platform.
    % The Tool-point will be influenced from both the relative motion of
    % the Stewart Platform and the actuation of the robotic joints
    % 
    % Forward Kinematics:
    %   Input:
    %       q     : Robot joint angular position (vector)
    %       q_t   : Robot joint angular velocity (vector)
    %       q_tt  : Robot joint angular acceleration (vector)
    %       
    %   Output:
    %       Pt     : Tool-point position (vector)
    %       Pt_t   : Tool-point velocity (vector)
    %       Pt_tt  : Tool-point acceleration (vector)
    %
    % Inverse Kinematics:
    %   Input: 
    %       Pt     : Tool-point position
    %       Pt_t   : Tool-point velocity
    %       Pt_tt  : Tool-point acceleration
    %   Output:
    %       q      : Robot joint angular position (vector)
    %       q_t    : Robot joint angular velocity (vector)
    %       q_tt   : Robot joint angular acceleration (vector)
    %
    % Stewart Motion:
    %   Input:
    %       q      : Robot joint angular position (vector)
    %       q_t    : Robot joint angular velocity (vector)
    %       q_tt   : Robot joint angular acceleration (vector)
    %       eta	   : Stewart platform position
    %       v	   : Stewart platform velocity
    %       v_t    : Stewart platform acceleration
    %   Output:
    %       Pt     : Tool-point position (relative to world coordinate)   
    %       Pt_t   : Tool-point velocity (relative to world coordinate)
    %       Pt_tt  : Tool-point acceleration (relative to world coordinate)
    
    %% Properties
    
    % Properties that can be changed during execution
    properties
    
    end
    
    % Properties that can only be changed before execution
    properties(Nontunable)
        StringChoice = 'Forward Kinematics'; % Comau Robot Mode
    end
    
    % Properties of the kinematic drop-down menu
    properties(Hidden, Constant)
        StringChoiceSet = matlab.system.StringSet({'Forward Kinematics', ...
            'Inverse Kinematics', 'Stewart Motion'});
    end
    
    % Only accessible by class members
    properties(Access = private)
        % Static link lengths
        a1 = 0.350;
        a2 = 1.160;
        a3 = 0.250;
        d1 = 0.830;
        d4 = 1.4922;
        d6 = 0.210;
        dt = 0.567;
        L;
        
        % Calibrated transformation matrices
        Hgn;    % World to EM8000  {g} -> {n}
        Hgq;    % World to EM1500  {g} -> {q}
        Hnq;	% EM8000 to EM1500 {n} -> {q}
        Hbr;    % EM8000 to Comau  {b} -> {r}
    end
    
    %% Methods for Simulink Interface
    
    methods (Access = protected)
        
        %% System block input-setup
        
        % Set number of inputs to the Simulink System object
        function num_inputs = getNumInputsImpl(obj)
            
            % Number of inputs to the related mode
            switch obj.StringChoice
                case {'Forward Kinematics'}
                    num_inputs = 3;
                case {'Inverse Kinematics'}
                    num_inputs = 3;
                case {'Stewart Motion'}
                    num_inputs = 6;
            end
        end
        
        % Set names of the inputs to the Simulink System object
        function varargout = getInputNamesImpl(obj)
            n = getNumInputsImpl(obj);  % Get number of inputs
            varargout = cell(1,n);      % Define function return vector
            
            % Set name of the inputs to the related mode
            switch obj.StringChoice
                case {'Forward Kinematics'}
                    varargout{1} = 'q';        % Set name of input port 1
                    varargout{2} = 'q_t';      % Set name of input port 2
                    varargout{3} = 'q_tt';     % Set name of input port 3
                case {'Inverse Kinematics'}
                    varargout{1} = 'P';        % Set name of input port 1
                    varargout{2} = 'P_t';      % Set name of input port 2
                    varargout{3} = 'P_tt';     % Set name of input port 3
                case {'Stewart Motion'}
                    varargout{1} = 'q';         % Set name of input port 1
                    varargout{2} = 'q_t';       % Set name of input port 2
                    varargout{3} = 'q_tt';      % Set name of input port 3

                    varargout{4} = 'eta';       % Set name of input port 4
                    varargout{5} = 'v';         % Set name of input port 5
                    varargout{6} = 'v_t';       % Set name of input port 6
                otherwise
                    % Error catch
                    msg = 'ERROR unknown kinematic type is chosen';
                    error(msg);
            end
        end
        
        %% System block output-setup
        
        % Set number of output ports to the Simulink System object
        function num_outputs = getNumOutputsImpl(~)
            num_outputs = 3;
        end
        
        % Set names of the output ports to the Simulink System object
        function varargout = getOutputNamesImpl(obj)
            n = getNumOutputsImpl(obj);	% Get number of outputs
            varargout = cell(1,n);      % Define function return vector
            
            % Set name of the outputs to the related mode
            switch obj.StringChoice
                case {'Forward Kinematics'}
                    varargout{1} = 'Pt';        % Set name of output port 1
                    varargout{2} = 'Pt_t';      % Set name of output port 2
                    varargout{3} = 'Pt_tt';     % Set name of output port 3
                case {'Inverse Kinematics'}
                    varargout{1} = 'q';         % Set name of output port 1
                    varargout{2} = 'q_t';       % Set name of output port 2
                    varargout{3} = 'q_tt';      % Set name of output port 3
                case {'Stewart Motion'}
                    varargout{1} = 'Pt';        % Set name of output port 1
                    varargout{2} = 'Pt_t';      % Set name of output port 2
                    varargout{3} = 'Pt_tt';     % Set name of output port 3
                otherwise
                    % Error catch
                    msg = 'ERROR unknown kinematic type is chosen';
                    error(msg);
            end
        end
        
        %% System output calculation

        % Initialize System object states
        % (One-time calculations)
        function setupImpl(obj)    
            % Dimensions
            obj.L = obj.d4 + obj.d6 + obj.dt;
            
            % Load calibration data
            obj.load_calibration();
        end
        
        % Reset System object states
        function resetImpl(obj)

        end
        
        % System output and state update equations
        function varargout = stepImpl(obj, varargin)
            
            % Switch-statement to determine the mode
            switch obj.StringChoice
                
                % Forward Kinematics
                case {'Forward Kinematics'}
                    % Define inputs:
                    q = varargin{1};
                    q_t = varargin{2};
                    q_tt = varargin{3};
                    
                    % Update states
                    [Pt, Pt_t, Pt_tt] = obj.forward(q, q_t, q_tt);
                    
                    % Define outputs
                    varargout{1} = Pt;
                    varargout{2} = Pt_t;
                    varargout{3} = Pt_tt;
                
                % Inverse Kinematics
                case {'Inverse Kinematics'}
                    % Define inputs:
                    Pt = varargin{1};
                    Pt_t = varargin{2};
                    Pt_tt = varargin{3};
                    
                    % Update states
                    [q, q_t, q_tt] = obj.inverse(Pt, Pt_t, Pt_tt);
                    
                    % Define outputs
                    varargout{1} = q;
                    varargout{2} = q_t;
                    varargout{3} = q_tt;
                    
                % Motion
                case {'Stewart Motion'}
                    % Define inputs:
                    q = varargin{1};
                    q_t = varargin{2};
                    q_tt = varargin{3};

                    eta = varargin{4};
                    v   = varargin{5};
                    v_t = varargin{6};
                    
                    % Calculate relative Tool-Point motion 
                    % {t}/{n} given in {n}
                    [Pt, Pt_t, Pt_tt] = obj.motion(q, q_t, q_tt, ...
                                                   eta, v, v_t);                     
                    
                    % Define outputs
                    % (Tool-point relative to Stewart Platform)
                    % {t}/{n} given in {n}
                    varargout{1} = Pt;      
                    varargout{2} = Pt_t;
                    varargout{3} = Pt_tt;
            end
        end
    end
    
    %% Methods for Matlab interface
    
    methods
    
        % Creates an constructor of the ComauRobot class
        function obj = ComauRobotSystem()
            obj.L = obj.d4 + obj.d6 + obj.dt;   % Horizontal length from 
                                                % q3 to Tool-point 
        end
        
        % Forward Kinematics
        function [Pt, Pt_t, Pt_tt] = forward(obj, q, q_t, q_tt)
            % Joint angular position
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            % Tool-point position (derived by "symbolic_forward")
            Pt = [
                -cos(q1)*(obj.L*sin(q3) + cos(q3)*obj.a3 - sin(q2)*obj.a2 - obj.a1);
                sin(q1)*(obj.L*sin(q3) + cos(q3)*obj.a3 - sin(q2)*obj.a2 - obj.a1);
                obj.L*cos(q3) - obj.a3*sin(q3) + cos(q2)*obj.a2 + obj.d1
                ];
            
            % Tool-point velocity
            J = obj.jacobian(q);
            Pt_t = J*q_t;
            
            % Tool-point acceleration
            J_t = obj.jacobian_dot(q, q_t);
            Pt_tt = J_t*q_t + J*q_tt;
            
        end
        
        % Inverse Kinematics
        function [q, q_t, q_tt] = inverse(obj, Pt, Pt_t, Pt_tt)
            
            Pt = [Pt; 1];
            % Position components
            xt = Pt(1);
            yt = Pt(2);
            zt = Pt(3);
            
            % Joint 1 angle
            q1 = -atan2(yt, xt);
            
            % Using the method of a Two-link planar robot
            % to find the last to joint angles
            
            % Transformation matrix {r -> j2}
            T1 = math3d.DH(-q1, obj.d1, obj.a1, pi/2); 
            Pq2t = math3d.InvH(T1)*Pt;     % TCP given in joint 2 {q2 -> t}
            xq2t = Pq2t(1);
            yq2t = Pq2t(2);
            
            % Geometry calculationsyy
            B = sqrt(obj.a3^2 + obj.L^2);
            C = sqrt(xq2t^2 + yq2t^2);
            D = (C^2 - obj.a2^2 - B^2) / (2*obj.a2*B);
            
            alpha = atan2(-sqrt(1-D^2), D);	% negative sign: elbow-up
            phi = atan2(obj.a3, obj.L);
            
            thetaA = atan2(yq2t, xq2t);
            thetaB = atan2(B*sin(alpha),obj.a2 + B*cos(alpha));
            
            % Joint 2 angle
            q2 = pi/2 - (thetaA - thetaB);
            
            % Joint 3 angle
            q3 = alpha - phi - q2;
            
            % Joint angular position
            q = [q1; q2; q3];
            
            % Joint angular velocity
            J = obj.jacobian(q);
            q_t = J\Pt_t;
            
            % Joint angular acceleration
            J_t = obj.jacobian_dot(q, q_t);
            q_tt = J\(Pt_tt - J_t*q_t);
            
        end
        
        % Calculate the motion of the system
        % relative to Stewart Platfrom neutral coordinate
        function [Pt, Pt_t, Pt_tt] = motion(obj, q, q_t, q_tt, ...
                                                 eta, v, v_t)
                                             
            % Body fixed velocity and acceleration skew matrices
            Sw = math3d.Skew(v(4:6));
            Sw_t = math3d.Skew(v_t(4:6));
            
            % Ship/Stewart relative to static csys {n} -> {b}
            Rnb = math3d.Rzyx(eta(4:6));
            Rnb_t = Rnb*Sw;
            Rnb_tt = Rnb*Sw*Sw + Rnb*Sw_t;
            
        	% Constant offset between stewart platform and robot base
            % {b} -> {r}
            r = obj.Hbr(1:3,4);
            Rbr = obj.Hbr(1:3,1:3);
            
            % Calculate Comau Robot forward kinematics
            % {r} -> {t}
            [P, P_t, P_tt] = obj.forward(q, q_t, q_tt);
            
            % Tool-point position relative to Stewart platform
            % {t}/{n} given in {n}
            Pt = eta(1:3) + Rnb*(r + Rbr*P);
            
            % Tool-point velocity relative to Stewart platform
            % {t}/{n} given in {n}
            Pt_t = v(1:3) + Rnb_t*(r + Rbr*P) + Rnb*(Rbr*P_t);
            
            % Tool-point acceleration relative to Stewart platform
            % {t}/{n} given in {n}
            Pt_tt = v_t(1:3) + Rnb_tt*(r + Rbr*P) ...
                   + 2*Rnb_t*(Rbr*P_t) + Rnb*(Rbr*P_tt);
        end
                                            
        % Jacobian
        function J = jacobian(obj, q)
            % Joint angular position
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            % Jacobian (derived by "symbolic_forward")
            J11 = sin(q1)*(obj.L*sin(q3) + cos(q3)*obj.a3 ...
                - sin(q2)*obj.a2 - obj.a1);
            J12 = cos(q1)*cos(q2)*obj.a2;
            J13 = -cos(q1)*(obj.L*cos(q3) - obj.a3*sin(q3));
            
            J21 = cos(q1)*(obj.L*sin(q3) + cos(q3)*obj.a3 ...
                - sin(q2)*obj.a2 - obj.a1);
            J22 = -sin(q1)*cos(q2)*obj.a2;
            J23 = sin(q1)*(obj.L*cos(q3) - obj.a3*sin(q3));
            
            J31 = 0;
            J32 = -sin(q2)*obj.a2;
            J33 = -obj.L*sin(q3) - cos(q3)*obj.a3;
            
            J = [J11, J12, J13;
                 J21, J22, J23;
                 J31, J32, J33];
        end
        
        % Jacobian time differentiated 
        function J_t = jacobian_dot(obj, q, q_t)
            % Joint angular position
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            % Joint angular velocity
            q1_t = q_t(1);
            q2_t = q_t(2);
            q3_t = q_t(3);
            
            % Jacobian_dot (derived by "symbolic_forward")
            J11_t = q1_t*cos(q1)*(obj.L*sin(q3) ...
                  + cos(q3)*obj.a3 - sin(q2)*obj.a2 - obj.a1) ...
                  - q2_t*sin(q1)*cos(q2)*obj.a2 ...
                  + q3_t*sin(q1)*(obj.L*cos(q3) - obj.a3*sin(q3));
            J12_t = -q1_t*sin(q1)*cos(q2)*obj.a2 ...
                  - q2_t*cos(q1)*sin(q2)*obj.a2;
            J13_t = q1_t*sin(q1)*(obj.L*cos(q3) - obj.a3*sin(q3)) ...
                  - q3_t*cos(q1)*(-obj.L*sin(q3) - cos(q3)*obj.a3);

            J21_t = -q1_t*sin(q1)*(obj.L*sin(q3) + cos(q3)*obj.a3 ...
                   - sin(q2)*obj.a2 - obj.a1) ...
                   - q2_t*cos(q1)*cos(q2)*obj.a2 ...
                   + q3_t*cos(q1)*(obj.L*cos(q3) - obj.a3*sin(q3)); 
            J22_t = -q1_t*cos(q1)*cos(q2)*obj.a2 ...
                   + q2_t*sin(q1)*sin(q2)*obj.a2;
            J23_t = q1_t*cos(q1)*(obj.L*cos(q3)- obj.a3*sin(q3)) ...
                  + q3_t*sin(q1)*(-obj.L*sin(q3) - cos(q3)*obj.a3);
                
            J31_t = 0;
            J32_t = -q2_t*cos(q2)*obj.a2;
            J33_t = q3_t*(-obj.L*cos(q3) + obj.a3*sin(q3)); 

            J_t = [J11_t, J12_t, J13_t;
                   J21_t, J22_t, J23_t;
                   J31_t, J32_t, J33_t];
        end
        
        function load_calibration(obj)
            % Load in calibration structure
            motionlab = load('calib.mat');
            
            % Transformation matrices found from calibration 
            obj.Hgn = motionlab.calib.WORLD_TO_EM8000.H;  % {g} -> {n}
            obj.Hgq = motionlab.calib.WORLD_TO_EM1500.H;  % {g} -> {q}
            obj.Hbr = motionlab.calib.EM8000_TO_COMAU.H;  % {b} -> {r}
        end
    end
end