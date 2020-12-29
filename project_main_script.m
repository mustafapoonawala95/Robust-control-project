%%%----------------Project script of Mustafa Poonawala (UFID 6109-7936)-- -%%%
%%%--------------Run the script in sections and not all at once------------
%%%-----------------Returning and running earlier sections after running---
%%%-------------later sections can cause errors due to variable sharing----
%% Defining the open loop plant.
% clear all
% clc
time = 0:0.001:15;

A_alpha = [-1.57     0.00      0.00     1       0.00];
A_beta  = [0.00      -0.5      0.17     0.00   -1.00];
A_p     = [-21.13    -2876.7   -2.1     -0.14  -0.05];
A_q     = [-82.92    -11.22    -0.01    -0.57  0.00];
A_r     = [-0.19     -11.86    -0.01    0.00   -0.57];
 
B_alpha  = [0.00      -0.1     0.00];
B_beta   = [-0.07     0.00     0.11];
B_p      = [-1234.7   -30.49   -1803.2];
B_q      = [-4.82     -119.65  -7.00];
B_r      = [14.84     0.27     -150.58];

 
C = eye(5);
D = zeros(5,3);

open_loop_states = {'alpha','beta','p','q','r'};
open_loop_inputs = {'del_a','del_e','del_r'};
open_loop_outputs = {'alpha','beta','p','q','r'};
open_loop = ss([A_alpha;A_beta;A_p;A_q;A_r],[B_alpha;B_beta;B_p;B_q;B_r],C,D,'StateName',open_loop_states,'InputName',open_loop_inputs,'OutputName',open_loop_outputs);
open_loop_step = step(open_loop,time);

figure;
subplot(2,1,1);
suptitle('Actual Short period dynamics with \delta_{e}');
plot(time,open_loop_step(:,1,2),'linewidth',2);
xlabel('Time[sec]');
ylabel('\alpha [rad]');
grid on;

subplot(2,1,2);
plot(time,open_loop_step(:,4,2),'linewidth',2);
xlabel('Time[sec]');
ylabel('q [rad/sec]');
grid on;


% figure;
% subplot(2,1,1);
% suptitle('Short period dynamics with \delta_{e}');
% plot(time,open_loop_step(:,1,2),'linewidth',2);
% xlabel('Time[sec]');
% ylabel('\alpha [rad]');
% grid on;
% 
% subplot(2,1,2);
% plot(time,open_loop_step(:,4,2),'linewidth',2);
% xlabel('Time[sec]');
% ylabel('q [rad/sec]');
% grid on;





% disp('Thse are the open loop plant eigen values.');
% modes = eig([A_alpha;A_beta;A_p;A_q;A_r]);
% disp(modes);
%% Extracting the short period dynamics. 
A_spd = [A_alpha(1), A_alpha(4);A_q(1),A_q(4)];
% A = [z_alpha_by_V0, 1+z_q_by_V0;
%      M_alpha, M_q];
% B = [z_delta_e_by_V0;
%      M_delta_e];
B_spd = [B_alpha(2);B_q(2)];
C_spd_alpha = [1 0];
C_spd_q = [0 1];
V0 = 2.5*967.51969;  % ft/sec    % Speed of sound at 40,000ft is 967.51969ft/sec 
z_alpha_by_V0 = -1.57;
z_delta_e_by_V0 = -0.1;
z_alpha = -1.57*V0;

z_delta_e = -0.1*V0;
C_spd_Az = [z_alpha 0];
D_spd_alpha = 0; 
D_spd_q = 0;
D_spd_Az = z_delta_e;
states_spd = {'alpha','q'};
inputs_spd = {'delta_achvd'};
outputs_spd = {'alpha','q','Az'};
z_alpha_by_V0 = A_spd(1,1);
z_q_by_V0 = A_spd(1,2)-1;
M_alpha = A_spd(2,1);
M_q = A_spd(2,2);
z_delta_e_by_V0 = B_spd(1,1);
M_delta_e = B_spd(2,1);

short_period_dynamics = ss(A_spd,B_spd,[eye(2);C_spd_Az],[0;0;D_spd_Az],'InputName',inputs_spd,'OutputName',outputs_spd,'StateName',states_spd);
%step(short_period_dynamics);
plant = ss(A_spd,B_spd,[C_spd_Az;[0 1]],[D_spd_Az;0],'InputName',inputs_spd,'OutputName',{'Az','q'},'StateName',states_spd);
%% Defining Actuator state space model.
Wn = 219.91148;
Wn_squared = (219.91148)^2;  %rad/seconds
zeta = 0.71;

A_ac = [0 1;
        -Wn_squared -2*zeta*Wn];
B_ac = [0;Wn_squared];
C_ac = [1 0];
D_ac = 0;
states_act = {'delta_achvd','delta_dot_achvd'};
inputs_act = {'delta_cmd'};
outputs_act = {'delta_achvd'};
Actuator = ss(A_ac,B_ac,C_ac,D_ac,'InputName',inputs_act,'OutputName',outputs_act,'StateName',states_act);

%%%-------------Defining the actual plant with the Actuator---------------
my_plant = plant*Actuator;
my_plant;

%% [FULL STATE FEEDBACK] Iterating over different penalty values and desiging 
% For faster execution the plotting within the loop can be 
% supressed. (line 231-240)

%time = 0:0.001:8;
A_tilde = [0 C_spd_Az;
           zeros(2,1), A_spd];
B_tilde = [D_spd_Az;
           B_spd];
qii = logspace(-8,-4,200);
%-------------------------------------------------

%K = zeros(length(qii),3);
iter = length(qii);
design_parameters_stfb = zeros(iter,4);
gain_marginsdB_stfb = zeros(iter,1);
gain_cross_w_stfb = zeros(iter,1);
stability_info_stfb = zeros(iter,2); 
phase_marginsdB_stfb = zeros(iter,1);
opt = stepDataOptions('StepAmplitude',32.174);
max_H_cosen_Az = zeros(length(qii),1);
max_H_sen_Az = zeros(length(qii),1);
max_H_cosen_Az = zeros(length(qii),1);
max_H_cosen_q = zeros(length(qii),1);
max_H_sen_q = zeros(length(qii),1);

for i = 1:length(qii)
    Q = [qii(i), 0, 0;
     0, 0, 0;
     0, 0, 0];
    H = lqr(A_tilde,B_tilde,Q,1);
    %K(i,:) = H;
    Kei = -H(1);
    K_alpha = -H(2);
    K_q = -H(3);
    
    A_complete_stfb = [z_alpha_by_V0, 1+z_q_by_V0, 0, z_delta_e_by_V0, 0;
                  M_alpha, M_q, 0, M_delta_e, 0;
                  z_alpha, 0, 0, z_delta_e, 0;
                  0, 0, 0, 0, 1;
                  Wn_squared*K_alpha, Wn_squared*K_q, Wn_squared*Kei, -Wn_squared, -2*zeta*Wn];

    B_complete_stfb = [0;0;-1;0;0];
    C_complete_stfb = [z_alpha, 0, 0, z_delta_e, 0;
              0,0,0,1,0;
              0,0,0,0,1];
    
    C_complete_stfb_sensitivity = [z_alpha, 0, 0, z_delta_e, 0;
                                   0,1,0,0,0];
                               
%     C_complete = [ 1,0,0,0,0,
%                    0,1,0,0,0;
%                   z_alpha, 0, 0, z_delta_e, 0];
    D_complete_stfb = [0;0;0];
    
    complete_plant_stfb_states = {'alpha','q','eI','dele_achvd','dele_achvd_dot'};
    complete_plant_stfb_inputs = {'dele_cmd'};
    complete_plant_stfb_outputs = {'Az','dele_achvd','dele_achvd_dot'};
    
    %%%---This section plots sensitivity and cosensitiviy plots.--------
    
    complete_plant_stfb_sensitivity_outputs = {'Az','q'};
    complete_plant_stfb_sensitivity = ss(A_complete_stfb,B_complete_stfb,C_complete_stfb_sensitivity,[0;0],'InputName',complete_plant_stfb_inputs,'OutputName',complete_plant_stfb_sensitivity_outputs,'StateName',complete_plant_stfb_states);
    complete_plant_stfb_sensitivity_tf = tf(complete_plant_stfb_sensitivity);
    
    ws = logspace(-1,3,300);
    H_cosen_Az = squeeze(mag2db(abs(freqresp(complete_plant_stfb_sensitivity_tf(1),ws)))); 
    max_H_cosen_Az(i) = max(H_cosen_Az);
    sen_Az = 1 - complete_plant_stfb_sensitivity_tf(1);
    H_sen_Az = squeeze(mag2db(abs(freqresp(sen_Az,ws))));
    max_H_sen_Az(i) = max(H_sen_Az);
    H_cosen_q = squeeze(mag2db(abs(freqresp(complete_plant_stfb_sensitivity_tf(2),ws))));
    max_H_cosen_q(i) = max(H_cosen_q);
    sen_q = 1 - complete_plant_stfb_sensitivity_tf(2);
    H_sen_q = squeeze(mag2db(abs(freqresp(sen_q,ws))));
    max_H_sen_q(i) = max(H_sen_q);
    
     
%-----------This section plots the sensitivity curves at each iteration.-------------    
%     figure(1);
%     title('Sensitivity plots[Az]','fontsize',14);
%     xlabel('ws[rad/sec]','fontsize',14);
%     ylabel('magnitude[dB]','fontsize',14);
%     semilogx(ws',squeeze(mag2db(abs(H_sen_Az))),'linewidth',2);
%     hold on;
%     grid on;
%     
%     figure(2);
%     title('Sensitivity plots[q]','fontsize',14);
%     xlabel('ws[rad/sec]','fontsize',14);
%     ylabel('magnitude[dB]','fontsize',14);
%     semilogx(ws',squeeze(mag2db(abs(H_sen_q))),'linewidth',2);
%     hold on;
%     grid on;
%     
%     figure(3);
%     title('Cosensitivity plots[Az]','fontsize',14);
%     xlabel('ws[rad/sec]','fontsize',14);
%     ylabel('magnitude[dB]','fontsize',14);
%     semilogx(ws',squeeze(mag2db(abs(H_cosen_Az))),'linewidth',2);
%     hold on;
%     grid on;
%     
%     figure(4);
%     title('Cosensitivity plots[q]','fontsize',14);
%     xlabel('ws[rad/sec]','fontsize',14);
%     ylabel('magnitude[dB]','fontsize',14);
%     semilogx(ws',squeeze(mag2db(abs(H_cosen_q))),'linewidth',2);
%     hold on;
%     grid on;
    
    %%%----------------------------------------------------------------------------
    
    complete_plant_stfb = ss(A_complete_stfb,B_complete_stfb,C_complete_stfb,D_complete_stfb,'InputName',complete_plant_stfb_inputs,'OutputName',complete_plant_stfb_outputs,'StateName',complete_plant_stfb_states);
    full_state_fb = step(complete_plant_stfb,time,opt);
    figure(5);
    plot(time,full_state_fb(:,1),'linewidth',2);
    grid on;
    xlabel('Time[s]','linewidth',2);
    ylabel('Az [ft/s2]','linewidth',2);
    
    title('Closed loop 1g step response for full state feedback','fontsize',14);
    hh = legend('A_z [ft/s^2]');
    set(hh, 'fontsize', 14, 'location', 'best');
    hold on;
    stability_info_stfb(i,1) = isstable(complete_plant_stfb);
    stability_info_stfb(i,2) = qii(i);
    
    over = 100*((max(full_state_fb(:,1))-32.1740)/32.1740);
    if over>0
        design_parameters_stfb(i,1) = over;
    else
        design_parameters_stfb(i,1) = 0;
    end

    under = -100*(min(full_state_fb(:,1))/32.1740);
     if under<=0
         design_parameters_stfb(i,2) = 0;
     else
         design_parameters_stfb(i,2) = under;
     end
    
    design_parameters_stfb(i,3) = (180/pi)*max(abs(full_state_fb(:,2)));   % Elevator displacement 
    design_parameters_stfb(i,4) = (180/pi)*max(abs(full_state_fb(:,3)));   % Elevator displacement rate.
    
    %------------------------FREQUENCY DOMAIN------------------------------
    short_period_dynamics_state_feedback = ss(A_spd,B_spd,[C_spd_Az;eye(2)],[D_spd_Az;0;0;]);
    my_plant_state_feedback = Actuator*short_period_dynamics_state_feedback;
    % Defining the controller.
    C_controller = [H(1)];
    D_controller = [0, H(2), H(3)];
    A_controller = [0];
    B_controller = [1, 0, 0];
    Controller_ss = ss(A_controller,B_controller,C_controller,D_controller);
    % defining analysis model.
    
    Lu_state_feedback = Controller_ss*my_plant_state_feedback;
    % Lu_tf = tf(Lu);
    %nyquist(Lu_state_feedback);
%     set(gca,'xlim',[-4 1], 'ylim', [-2.5 2.5]);
%     set(gcf,'color','w')
    
    %nyquist(Lu);
    J = allmargin(Lu_state_feedback);
    gain_mar = mag2db(J.GainMargin);
    [M,I] = min(abs(gain_mar));
    gain_marginsdB_stfb(i) = gain_mar(I);
    gain_cross = J.PMFrequency;
    gain_cr = gain_cross(gain_cross>0 & gain_cross<inf);    
    gain_cross_w_stfb(i) = min(gain_cr);
    
    phase_marginsdB_stfb(i) = min(J.PhaseMargin);
   % pause(0.2);
end
%% Creating the design charts.

figure;
subplot(2,2,2);
plot(gain_cross_w_stfb,design_parameters_stfb(:,2),'-o','linewidth',2);
title('Plot of undershoot.');
ylabel('% undershoot','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;


subplot(2,2,1);
plot(gain_cross_w_stfb,design_parameters_stfb(:,1),'-o','linewidth',2);
title('Plot of overshoot.');
ylabel('% overershoot','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,2,3);
plot(gain_cross_w_stfb,design_parameters_stfb(:,3),'-o','linewidth',2);
title('Max elevator deflection.');
ylabel('Elevator displacement [deg/g]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,2,4);
plot(gain_cross_w_stfb,design_parameters_stfb(:,4),'-o','linewidth',2);
title('Max elevator deflection rate.');
ylabel('Elevator displacement rate [deg/sec*g]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

figure;
subplot(2,1,1);
plot(gain_cross_w_stfb,gain_marginsdB_stfb,'-o','linewidth',2);
title('Gain margins');
ylabel('dB','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,1,2);
plot(gain_cross_w_stfb,phase_marginsdB_stfb,'-o','linewidth',2);
title('Phase margins');
ylabel('degree','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

figure;
plot(gain_cross_w_stfb,qii,'-o','linewidth',2);
title('eI penalty vs gain cross over frequency');
ylabel('value','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

figure;
subplot(2,1,1);
suptitle('Max Sensitivity, Cosensitivity [Az]');
plot(gain_cross_w_stfb,max_H_sen_Az,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,1,2);
plot(gain_cross_w_stfb,max_H_cosen_Az,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

figure;
subplot(2,1,1);
suptitle('Max Sensitivity, Cosensitivity [q]');
plot(gain_cross_w_stfb,max_H_sen_q,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,1,2);
plot(gain_cross_w_stfb,max_H_cosen_q,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;



%% Observer design.
% Defining necessary matrices for finding observer gains. 
gain_cross_stfb_design = 20.75;  %<<--------Identifying design point through gain cross over frequency.
%qii_design = 1e-09;

% Searching for the penalty value for the design point gain crossover frequency
%------------------------------------------------------------------------------
init = inf;
for i=1:length(gain_cross_w_stfb)
    if abs(gain_cross_w_stfb(i)-gain_cross_stfb_design) < init
        indx = i;
        init = abs(gain_cross_w_stfb(i)-gain_cross_stfb_design);
    end
end
qii_design = qii(indx);
%------------------------------------------------------------------------------

Q = [qii_design 0   0;
     0          0   0;
     0          0   0];
K = lqr(A_tilde,B_tilde,Q,1); % Recalculating K for debugging purposes.
C_tilde = [1 0 0;
           0 0 1];
       
h = logspace(-3,4,150);
%----------------Preallocating design parameters---------------------------
%--------------------------------------------------------------------------
iterations = length(h);
design_parameters = zeros(iterations,4);
gain_marginsdB = zeros(iterations,1);
gain_cross_w = zeros(iterations,1);
phase_marginsdB = zeros(iterations,1);
stability_info = zeros(iterations,2);
max_H_cosen_Az = zeros(iterations,1);
max_H_sen_Az = zeros(iterations,1);
max_H_cosen_Az = zeros(iterations,1);
max_H_cosen_q = zeros(iterations,1);
max_H_sen_q = zeros(iterations,1);
delta_w = zeros(iterations,1);
%--------------------------------------------------------------------------
for g = 1:(iterations)
    W = eye(3)*h(g);
    V = [1 0;
        0 0.1];
    L = lqr(A_tilde',C_tilde',W,V);
    L = L';

    A_I = 0;
    B_I1 = [1 0];
    B_I2 = [-1];
    C_I = [1;0];
    D_I1 = [0 0 ;0 1];
    D_I2 = [0;0];
    B_cmd = [-1;0;0];
    LCI = L*C_I;
    A0_cl = (A_tilde - B_tilde*K - L*C_tilde);
    zero = zeros(1,3);

    % Defining the dynamic compensator now.
    K = lqr(A_tilde,B_tilde,Q,1);
    A_compensator = [A_I zero;
                     LCI A0_cl];
   
    B1_compensator = [B_I1;
                      L*D_I1]; 

    B2_compensator = [B_I2;
                      B_cmd];

    C_compensator = [0 -K];

    D1_compensator = [0 0];
    D2_compensator = 0;
    
    %----------------------------------------------------------------------
    %%%----------Closing the loop with actuator and compensator------------
    %----------------------------------------------------------------------

    A_complete = [short_period_dynamics.A short_period_dynamics.B zeros(2,5);
                  zeros(2,2) Actuator.A Actuator.B*[0 -K];
                  B1_compensator*my_plant.C (A_compensator + B1_compensator*my_plant.D*[0 -K])];

    B_complete = [zeros(4,1);
                  B2_compensator];
    C_complete = [eye(4) zeros(4,4);
                  zeros(1,6) 1 0;
                  zeros(1,7) 1;
                  C_spd_Az D_spd_Az zeros(1,5)];

    D_complete = zeros(7,1);
    complete_states = {'alpha','q','u_achvd','u_dot_achvd','eI_Az','eI_hat_Az','alpha_hat','q_hat'};
    complete_inputs = {'Az_command'};
    complete_outputs = {'alpha','q','u_achvd','u_dot_achvd','alpha_hat','q_hat','Az'};

    complete_plant = ss(A_complete,B_complete,C_complete,D_complete,'InputName',complete_inputs,'OutputName',complete_outputs,'StateName',complete_states);
    opt2 = stepDataOptions('StepAmplitude',32.174);
    
    stability_info(g,1) = isstable(complete_plant);
    stability_info(g,2) = h(g);
    step_output = step(complete_plant,opt2,time);
    plot(time,step_output(:,7),'linewidth',2);
    grid on;
    xlabel('Time[s]','linewidth',2);
    ylabel('Az [ft/s2]','linewidth',2);
    title('Closed loop 1g step response with actuator dynamics and observer','fontsize',14);
    hh = legend('A_z [ft/s^2]');
    set(hh, 'fontsize', 14, 'location', 'best');
    hold on;
    
    over = 100*((max(step_output(:,7))-32.1740)/32.1740);
    if over>0
        design_parameters(g,1) = over;
    else
        design_parameters(g,1) = 0;
    end

    under = -100*(min(step_output(:,7))/32.1740);
     if under<=0
         design_parameters(g,2) = 0;
     else
         design_parameters(g,2) = under;
     end
    
    design_parameters(g,3) = (180/pi)*max(abs(step_output(:,3)));    % Elevator displacement 
    design_parameters(g,4) = (180/pi)*max(abs(step_output(:,4)));    % Elevator displacement rate.
    
   %-------------This section collects sensitivity and co sensitivity data-
    
   % complete_states = {'alpha','q','u_achvd','u_dot_achvd','eI_Az','eI_hat_Az','alpha_hat','q_hat'};
   % complete_inputs = {'Az_command'};
    %complete_outputs = {'alpha','q','u_achvd','u_dot_achvd','alpha_hat','q_hat','Az'};
    complete_sensitivity_outputs = {'Az','q'};
    C_complete_sensitivity = [C_spd_Az D_spd_Az zeros(1,5);
                              0,1,0,0,0,0,0,0];
                          
    complete_plant_sensitivity = ss(A_complete,B_complete,C_complete_sensitivity,[0;0],'InputName',complete_inputs,'OutputName',complete_sensitivity_outputs,'StateName',complete_states);
    complete_plant_sensitivity_tf = tf(complete_plant_stfb_sensitivity);
    
    ws = logspace(-1,3,300);
    H_cosen_Az = squeeze(mag2db(abs(freqresp(complete_plant_sensitivity_tf(1),ws)))); 
    max_H_cosen_Az(g) = max(H_cosen_Az);
    sen_Az = 1 - complete_plant_sensitivity_tf(1);
    H_sen_Az = squeeze(mag2db(abs(freqresp(sen_Az,ws))));
    max_H_sen_Az(g) = max(H_sen_Az);
    H_cosen_q = squeeze(mag2db(abs(freqresp(complete_plant_sensitivity_tf(2),ws))));
    max_H_cosen_q(g) = max(H_cosen_q);
    sen_q = 1 - complete_plant_sensitivity_tf(2);
    H_sen_q = squeeze(mag2db(abs(freqresp(sen_q,ws))));
    max_H_sen_q(g) = max(H_sen_q);
    
    %%%---------------------Finding the loop gain-----------------------%%%
    compensator_states = {'eI_Az','eI_Az_hat','alpha_hat','q_hat'};
    compensator_inputs = {'Az','q',};
    compensator_outputs = {'delta_cmd'};
    compensator = ss(A_compensator,B1_compensator,-C_compensator,-D1_compensator,'StateName',compensator_states,'InputName',compensator_inputs,'OutputName',compensator_outputs);
    
    Lu = compensator*my_plant;
%     margin(Lu);
%     hold on;
    J = allmargin(Lu);
    gain_mar = mag2db(J.GainMargin);
    [M,I] = min(abs(gain_mar));
    gain_marginsdB(g) = gain_mar(I);
    phase_marginsdB(g) = min(J.PhaseMargin);
    gain_cross = J.PMFrequency;
    gain_cr = gain_cross(gain_cross>0 & gain_cross<inf);    
    gain_cross_w(g) = min(gain_cr);
    
    %pause;
    delta_w(g) = (1/(2*pi))*(gain_cross_stfb_design - gain_cross_w(g));
    
end
%% Creating the design charts.

figure;
subplot(2,2,2);
plot(gain_cross_w,design_parameters(:,2),'-o','linewidth',2);
title('Plot of undershoot.');
ylabel('% undershoot','fontsize',14);
grid on;


subplot(2,2,1);
plot(gain_cross_w,design_parameters(:,1),'-o','linewidth',2);
title('Plot of overshoot.');
ylabel('% overershoot','fontsize',14);
grid on;

subplot(2,2,3);
plot(gain_cross_w,design_parameters(:,3),'-o','linewidth',2);
title('Max elevator deflection.');
ylabel('Elevator displacement [deg/g]','fontsize',14);
grid on;

subplot(2,2,4);
plot(gain_cross_w,design_parameters(:,4),'-o','linewidth',2);
title('Max elevator deflection rate.');
ylabel('Elevator displacement rate [deg/sec*g]','fontsize',14);
grid on;

figure;
subplot(2,1,1);
plot(gain_cross_w,gain_marginsdB,'-o','linewidth',2);
title('Gain margins');
ylabel('dB','fontsize',14);
grid on;

subplot(2,1,2);
plot(gain_cross_w,phase_marginsdB,'-o','linewidth',2);
title('Phase margins');
ylabel('degree','fontsize',14);
grid on;

figure;
plot(gain_cross_w,h,'-o','linewidth',2);
title('observer penalty vs gain cross over frequency');
ylabel('value','fontsize',14);
xlabel('rad/sec','fontsize',14);
grid on;

figure;
subplot(2,1,1);
suptitle('Max Sensitivity, Cosensitivity [Az]');
plot(gain_cross_w,max_H_sen_Az,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,1,2);
plot(gain_cross_w,max_H_cosen_Az,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

figure;
subplot(2,1,1);
suptitle('Max Sensitivity, Cosensitivity [q]');
plot(gain_cross_w,max_H_sen_q,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

subplot(2,1,2);
plot(gain_cross_w,max_H_cosen_q,'-o','linewidth',2);
ylabel('magnitude [dB]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;

figure(9);
plot(gain_cross_w,delta_w,'-o','linewidth',2);
title('Difference between gain cross over frequency of LQR and Dynamic compensator');
ylabel('\Delta_{\omega}[Hz]','fontsize',14);
xlabel('Cross over frequency [rad/sec]','fontsize',14);
grid on;





%% Closing the loop for the design point. 
%%%----------------------------------------------------------------------------------------------------%%
%%%---[This section plots the response for the design point and shows that design parameters are met]
%%%----Check command window for design parameters------------------------------------------------------%%
%%%----------------------------------------------------------------------------------------------------%%
qii_design;
h_design = 3777;

A_tilde_design = [0 C_spd_Az;
                  zeros(2,1), A_spd];
B_tilde_design = [D_spd_Az;
                  B_spd];

C_tilde_design = [1 0 0;
                  0 0 1];

Q_design = [qii_design 0   0;
     0          0   0;
     0          0   0];
K_design = lqr(A_tilde_design,B_tilde_design,Q_design,1);


W_design = eye(3)*h_design;
V = [1 0;
    0 0.1];
L_design = lqr(A_tilde_design',C_tilde_design',W_design,V);
L_design = L_design';


A_I = 0;
B_I1 = [1 0];
B_I2 = [-1];
C_I = [1;0];
D_I1 = [0 0 ;0 1];
D_I2 = [0;0];
B_cmd = [-1;0;0];
LCI_design = L_design*C_I;
A0_cl = (A_tilde_design - B_tilde_design*K_design - L_design*C_tilde_design);
zero = zeros(1,3);

% Defining the dynamic compensator now.

A_compensator_design = [A_I zero;
                 LCI_design A0_cl];

B1_compensator_design = [B_I1;
                  L_design*D_I1]; 

B2_compensator_design = [B_I2;
                  B_cmd];

C_compensator_design = [0 -K_design];

D1_compensator_design = [0 0];
D2_compensator_design = 0;




A_complete_design = [short_period_dynamics.A short_period_dynamics.B zeros(2,5);
              zeros(2,2) Actuator.A Actuator.B*[0 -K_design];
              B1_compensator_design*my_plant.C (A_compensator_design + B1_compensator_design*my_plant.D*[0 -K_design])];

B_complete_design = [zeros(4,1);
              B2_compensator_design];
C_complete_design = [eye(4) zeros(4,4);
                     zeros(1,6) 1 0;
                     zeros(1,7) 1;
                     C_spd_Az D_spd_Az zeros(1,5)];

D_complete_design = zeros(7,1);
complete_states = {'alpha','q','u_achvd','u_dot_achvd','eI_Az','eI_hat_Az','alpha_hat','q_hat'};
complete_inputs = {'Az_command'};
complete_outputs = {'alpha','q','u_achvd','u_dot_achvd','alpha_hat','q_hat','Az'};

complete_plant_design = ss(A_complete_design,B_complete_design,C_complete_design,D_complete_design,'InputName',complete_inputs,'OutputName',complete_outputs,'StateName',complete_states);
opt2 = stepDataOptions('StepAmplitude',32.174);

%--------------------------------------------------------------------------
%-------------------Collecting step response data--------------------------
%--------------------------------------------------------------------------
step_design_point = step(complete_plant_design,opt2,time);
final_design_parameters = zeros();
%-------------------------------------------------------------------------%
over = 100*((max(step_design_point(:,7))-32.1740)/32.1740);
if over>0
    final_design_parameters(1) = over;
else
    final_design_parameters(1) = 0;
end

under = -100*(min(step_output(:,7))/32.1740);
 if under<=0
     final_design_parameters(2) = 0;
 else
     final_design_parameters(2) = under;
 end

% final_design_parameters(3) = (180/pi)*max(abs(step_output(:,3)));    % Elevator displacement 
% final_design_parameters(4) = (180/pi)*max(abs(step_output(:,4)));    % Elevator displacement rate.

%%%------------------------------------------------------------------------
%%%---------------------Finding the loop gain---------------------------%%%
%%%------------------------------------------------------------------------
compensator_states = {'eI_Az','eI_Az_hat','alpha_hat','q_hat'};
compensator_inputs = {'Az','q',};
compensator_outputs = {'delta_cmd'};
compensator_design = ss(A_compensator_design,B1_compensator_design,-C_compensator_design,-D1_compensator_design,'StateName',compensator_states,'InputName',compensator_inputs,'OutputName',compensator_outputs);

Lu_design = compensator_design*my_plant;
%     margin(Lu);
%     hold on;
J = allmargin(Lu_design);
gain_mar_design = mag2db(J.GainMargin);
[M,I] = min(abs(gain_mar_design));
gain_marginsdB_design = gain_mar_design(I);
final_design_parameters(3) = gain_marginsdB_design;
%disp(gain_marginsdB_design);
phase_marginsdB_design = min(J.PhaseMargin);
final_design_parameters(4) = phase_marginsdB_design;
disp(phase_marginsdB_design);
gain_cross = J.PMFrequency;
gain_cr = gain_cross(gain_cross>0 & gain_cross<inf);    
gain_cross_w_design = min(gain_cr);

%pause;
%delta_w(g) = gain_cross_stfb_design - gain_cross_w(g);


disp('The overshoot, undershoot, gain margin and phase margin of the design point respectively are:');
disp(final_design_parameters);


plot(time,step_design_point(:,7),'linewidth',2);
grid on;
xlabel('Time[s]','linewidth',2);
ylabel('Az [ft/s2]','linewidth',2);
title('Closed loop step response with actuator dynamics and observer','fontsize',14);
hh = legend('A_z [ft/s^2]');
set(hh, 'fontsize', 14, 'location', 'best');
hold on;

%%%-----------Plots of actual vs estimated alpha and q are supressed.------ 
% figure;
% plot(time,step_design_point(:,1),time,step_design_point(:,5),'linewidth',2);
% grid on;
% xlabel('Time[s]','linewidth',2);
% ylabel('rad','linewidth',2);
% title('Closed loop step response with actuator dynamics and observer','fontsize',14);
% hh = legend('\alpha','\alpha_{hat}');
% set(hh, 'fontsize', 14, 'location', 'best');
% hold on;
% 
% figure;
% plot(time,step_design_point(:,2),time,step_design_point(:,6),'linewidth',2);
% grid on;
% xlabel('Time[s]','linewidth',2);
% ylabel('rad/sec','linewidth',2);
% title('Closed loop step response with actuator dynamics and observer','fontsize',14);
% hh = legend('q','q_{hat}');
% set(hh, 'fontsize', 14, 'location', 'best');
% hold on;

%% --------------------------Guidance for the design point.------------------------------%%
%%%------------Deleting all but necessary variables for guidance-----------
save('A_complete_design.MAT','A_complete_design');
save('B_complete_design.MAT','B_complete_design');
C_complete_design_g = [eye(8);
                       C_complete_design(7,:)];
D_complete_design_g = zeros(9,1);
save('C_complete_design_g.MAT','C_complete_design_g');
save('D_complete_design_g.MAT','D_complete_design_g');

clear all 
clc 

load('A_complete_design.MAT');
load('B_complete_design.MAT');
load('C_complete_design_g.MAT');
load('D_complete_design_g.MAT');
%%%------------------------------------------------------------------------
%%%------------------------------------------------------------------------
%% ------------------------------------------------------------------
% State is y = [beta, RT1, RT2, RM1, RM2, VT1, VT2, VM1, VM2]
%
% beta - target flight path angle
% RT1  - horizontal position of target wrt inertial cs
% RT2  - vertical position of target wrt inertial cs
% RM1  - horizontal position of missile wrt inertial cs
% RT2  - horizontal position of missile wrt inertial cs
% VT1  - horizontal velocity of target wrt inertial cs
% VT2  - vertical velocity of target wrt inertial cs
% VM1  - horizontal velocity of missile wrt inertial cs
% VM2  - vertical velocity of missile wrt inertial cs

%clear all

% Define time vector
t0 = 0; 
tf = 8.9;  % simulate for 9 seconds for better miss distance.

% Parameters
nT = 3*32.2;                      % 3g evasive maneuver.
HE_rad = -20*pi/180;              % -20 degree heading error.
Np = 4.5;
%minimum_mis_distance = zeros(length(Np));
%miss_dis = inf;
linespecs = {'b-', 'b-.','b-x','b-sq','b-+','b-^'};

%---------------- Defining the closep loop matrices------------------------------------

%---------------------------------------------------------------------------------------                

for ii = 1 : length(Np)
    
    Np_current = Np(ii);

    % Initial Conditions
    
    beta_rad = 0;
    RT1      = 20000;
    RT2      = 40000;
    RM1      = 0;
    RM2      = 40000;
    VM       = 2.5*967.51969;      % Speed of sound at 40,000ft is 967.51969ft/sec.
    VT       = 300;                % Target drone approaching head on with velocity 300ft/sec.
    VT1      = -VT*cos(beta_rad);
    VT2      =  VT*sin(beta_rad);

    % relative positions and velocities
    RTM1 = RT1 - RM1;
    RTM2 = RT2 - RM2;

    % relative distance
    RTM = sqrt(RTM1^2 + RTM2^2);
    
    % line of sight angle and time derivative
    lambda = atan2( RTM2, RTM1 );

    % missile lead angle
    L = asin( VT*sin( beta_rad + lambda )/VM );

    % missile velocity components
    VM1  = VM*cos(lambda + L + HE_rad);
    VM2  = VM*sin(lambda + L + HE_rad);

    % Pointers to states [beta, RT1, RT2, RM1, RM2, VT1, VT2, VM1, VM2]
    sel_beta = 1;
    sel_RT1  = 2;
    sel_RT2  = 3;
    sel_RM1  = 4;
    sel_RM2  = 5;
    sel_VT1  = 6;
    sel_VT2  = 7;
    sel_VM1  = 8;
    sel_VM2  = 9;
    
    % Initial condition vector
    X_plant = zeros(1,8);
    y0 = [beta_rad, RT1, RT2, RM1, RM2, VT1, VT2, VM1, VM2,X_plant]';

    options = odeset('abstol', 1e-13, 'reltol', 1e-10);

    % Integrate nonlinear 2-D engagement situation
    [t,y] = ode45(@nlinpronav, [t0, tf], y0, options, HE_rad, Np_current, A_complete_design, B_complete_design, C_complete_design_g, D_complete_design_g, nT);

    % Missile and target positions in inertial coordinate system
    figure(1)
    plot(y(:,sel_RM1), y(:,sel_RM2), '--', 'linewidth', 2); hold on
    xlabel('Downrange [ft]', 'fontsize', 14);
    ylabel('Crossrange [ft]', 'fontsize', 14);
    set(gca, 'fontsize', 14);
    set(gcf, 'color', 'w');
    grid on
    
    % target and missile velocity magnitudes
    VT = sqrt( y(:,sel_VT1).^2 + y(:,sel_VT2).^2 );

    % relative positions and velocities
    RTM1 = y(:,sel_RT1) - y(:,sel_RM1);
    RTM2 = y(:,sel_RT2) - y(:,sel_RM2);
    VTM1 = y(:,sel_VT1) - y(:,sel_VM1);
    VTM2 = y(:,sel_VT2) - y(:,sel_VM2);

    % relative distance
    RTM = sqrt(RTM1.^2 + RTM2.^2);
    [minRTM,idx] = min(RTM);
    % line of sight angle and time derivative
    lambda     = atan2( RTM2, RTM1 );
    lambda_dot = (RTM1.*VTM2 - RTM2.*VTM1)./RTM.^2;

    % closing velocity
    VC = -(RTM1.*VTM1 + RTM2.*VTM2)./RTM;

    % Compute acc commands
    nc = Np_current*VC.*lambda_dot;
    f = size(y);
    f(1);
    Az_achieved = zeros(f(1),1);
  
    for k =1:f(1)
        Az_achieved(k) = C_complete_design_g(9,:)*y(k,10:17)' + D_complete_design_g(9,:)*nc(k);
    end
    
    figure(2)
    plot(t, nc./32.2,t,(Az_achieved/32.2),'--', 'linewidth', 2); hold on 
    
end

figure(1)
h = legend('Missile','Drone','Missile','Drone');
set(h,'location','best');
plot(y(:,sel_RT1), y(:,sel_RT2), 'r--', 'linewidth', 2); 
plot(y(1,sel_RM1), y(1,sel_RM2), 'ob', 'linewidth', 2);
plot(y(1,sel_RT1), y(1,sel_RT2), 'or', 'linewidth', 2);

figure(2)
xlabel('Time [s]', 'fontsize', 14);
ylabel('Acceleration [G]', 'fontsize', 14);
title('3 G Target Maneuver','fontsize',14);
set(gca, 'fontsize', 14, 'xlim', [0 10]);
set(gcf, 'color', 'w');
grid on
h = legend('Az_{cmd}','Az_{achieved}');
set(h,'location','best');


dele_deg = (180/pi)*y(:,12);
figure(3);
plot(t,dele_deg,'linewidth',2);
title('Elevator displacement','fontsize',14);
ylabel('[degrees]','fontsize', 14);
xlabel('Time [s]', 'fontsize', 14);
grid on;


dele_rate_deg = (180/pi)*y(:,13);
figure(4);
plot(t,dele_rate_deg,'linewidth',2);
title('Elevator displacement rate','fontsize',14);
ylabel('[degrees/sec]','fontsize', 14);
xlabel('Time [s]', 'fontsize', 14);
grid on;

figure(5);
plot(t,y(:,10),t,y(:,16),'linewidth',2);
title('\alpha Vs \alpha_{hat} ','fontsize',14);
ylabel('rad','fontsize', 14);
xlabel('Time [s]', 'fontsize', 14);
h = legend('\alpha','\alpha_{hat}');
set(h,'location','best');
grid on;

figure(6);
plot(t,y(:,11),t,y(:,17),'linewidth',2);
title('q Vs q_{hat} ','fontsize',14);
ylabel('rad/sec]','fontsize', 14);
xlabel('Time [s]', 'fontsize', 14);
h = legend('q','q_{hat}');
set(h,'location','best');
grid on;

figure(7);
plot(t,y(:,14),t,y(:,15),'linewidth',2);
title('eIAz Vs eIAz_{hat} ','fontsize',14);
ylabel('ft/sec^2','fontsize', 14);
xlabel('Time [s]', 'fontsize', 14);
h = legend('eIAz Vs eIAz_{hat}');
set(h,'location','best');
grid on;

figure(8);
plot(y(idx,sel_RM1), y(idx,sel_RM2), 'o',y(idx,sel_RT1), y(idx,sel_RT2), '*', 'linewidth', 2); 
hold on;
a = num2str(RTM(idx));
title(['Miss distance = ',a,'ft'],'fontsize',14);
h = legend('Missile Position','Target Position');
set(h,'location','best');
xlabel('Downrange [ft]', 'fontsize', 14);
ylabel('Crossrange [ft]', 'fontsize', 14);
set(gca, 'fontsize', 14);
set(gcf, 'color', 'w');
grid on

%%%----------------------------------------------------------------------%%
%------------------------------THE END-----------------------------------%%
%%%----------------------------------------------------------------------%%








