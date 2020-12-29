% State is y = [beta, RT1, RT2, RM1, RM2, VT1, VT2, VM1, VM2]
% Parameters are nT
%
function dy = nlinpronav(t, y, He_rad, Np, A_complete_design, B_complete_design, C_complete_design_g, D_complete_design_g, nT)


% Define pointers to state variables
%--------------------------------------------------------------------------
% Pointers to states
sel_beta = 1;
sel_RT1  = 2;
sel_RT2  = 3;
sel_RM1  = 4;
sel_RM2  = 5;
sel_VT1  = 6;
sel_VT2  = 7;
sel_VM1  = 8;
sel_VM2  = 9;

% Preallocate solution vector
%--------------------------------------------------------------------------
dy = [0; 0; 0; 0; 0; 0; 0; 0; 0;zeros(8,1)];  % Including the plant states. 

% Preliminary terms to compute DE RHS
%--------------------------------------------------------------------------

% target and missile velocity magnitudes
VT = sqrt( y(sel_VT1)^2 + y(sel_VT2)^2);
VM = sqrt(y(sel_VM1)^2 + y(sel_VM2)^2);
% relative positions and velocities
RTM1 = y(sel_RT1) - y(sel_RM1);
RTM2 = y(sel_RT2) - y(sel_RM2);
VTM1 = y(sel_VT1) - y(sel_VM1);
VTM2 = y(sel_VT2) - y(sel_VM2);

% relative distance
RTM = sqrt(RTM1^2 + RTM2^2);

% line of sight angle and time derivative
lambda     = atan2( RTM2, RTM1 );
lambda_dot = (RTM1*VTM2 - RTM2*VTM1)/RTM^2;

% closing velocity
VC = -(RTM1*VTM1 + RTM2*VTM2)/RTM;

% command acceleration (True Proportional Navigation)
nc = Np*VC*lambda_dot;



% Using pronav output as plant input.
Az_reg = nc;                         %Converting a_true to Az_reg. The assumption is that a_true is entireably achievable by pursuer.   
dy(10:17) = A_complete_design*y(10:17) + B_complete_design*Az_reg;


% DE RHS computations y = [beta, RT1, RT2, RM1, RM2, VT1, VT2, VM1, VM2]
%--------------------------------------------------------------------------
dy(1) =  nT/VT;
dy(2) = -VT*cos(y(sel_beta));
dy(3) =  VT*sin(y(sel_beta));
dy(4) =  y(sel_VM1);
dy(5) =  y(sel_VM2);
dy(6) =  nT*sin(y(sel_beta));
dy(7) =  nT*cos(y(sel_beta));
Az_achieved = C_complete_design_g(9,:)*y(10:17) + D_complete_design_g(9,:)*Az_reg; % Calculating the achieved Az through plant dynamics.


dy(8) = -Az_achieved*sin(lambda);    
dy(9) =  Az_achieved*cos(lambda);   