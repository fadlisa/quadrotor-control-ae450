clc; clear;
close all

addpath('/home/fadllocal/Documents/Research Work/quadrotor-control-ae450/drone-lib');

%% DEFINE
R2D=180/pi;
D2R=pi/180;

%% INIT. PARAMS.
BeeBotV2_params=containers.Map({'mass','armLength','Ixx','Iyy','Izz','Ixy','Ixz','Iyz'}, ...
    {0.60043, 0.1454, 0.002195, 0.002557, 0.004148, 0.00002739, 0.00009406, 0.00000479071});

BeeBotV2_initStates=[0,0,-6, ... %X,Y,Z (x1, x2, x3)
    0,0,0, ... %phi, theta, psi (x7, x8, x9) was: Xdot, Ydot, Zdot
    0,0,0, ... %Xdot, Ydot, Zdot (x4, x5, x6) was: phi, theta, psi
    0,0,0]';   %p, q, r (x10, x11, x12)

BeeBotV2_initInputs=[0,0,0,0]'; %u1,u2,u3,u4 (T,M1,M2,M3)

BeeBotV2_body=[0.1454*cos(pi/4),  0.1454*cos(pi/4), 0,  1;...   %arm1
               -0.1454*cos(pi/4),  0.1454*cos(pi/4), 0,  1;...   %arm2
               -0.1454*cos(pi/4),  -0.1454*cos(pi/4), 0,  1;...   %arm3
               0.1454*cos(pi/4),  -0.1454*cos(pi/4), 0,  1;...   %arm4
                0,                  0,              0,  1;...
                0,                  0,          -0.15,  1]';

BeeBotV2_gains=containers.Map(...
    {'P_phi','I_phi','D_phi',...        %roll PID
    'P_theta','I_theta','D_theta',...   %pitch PID
    'P_psi','I_psi','D_psi',...         %yaw PID
    'P_zdot','I_zdot','D_zdot'},...     %altitude PID
    {0.5,0.0,0.15,...
    2,0.0,0.15,...
    0.4,0.0,0.3,...
    10,0.2,0.0});
    
simulationTime=2;

BeeBotV2=Drone(BeeBotV2_params,BeeBotV2_initStates,BeeBotV2_initInputs,BeeBotV2_gains,simulationTime);

%% INIT. 3D Fig.
fig1=figure('pos',[0 200 800 800]);
h=gca;
view(3);
fig1.CurrentAxes.ZDir='Reverse';
fig1.CurrentAxes.YDir='Reverse';

axis equal;
grid on;

xlim([-5 5]); ylim([-5 5]); zlim([-8 0]);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

hold(gca, 'on');
BeeBotV2_state=BeeBotV2.GetState();
wHb = [RPY2Rot(BeeBotV2_state(4:6))' BeeBotV2_state(1:3); 0 0 0 1];
BeeBotV2_world=wHb*BeeBotV2_body;
BeeBotV2_atti=BeeBotV2_world(1:3, :);

fig1_ARM13=plot3(gca,BeeBotV2_atti(1,[1 3]),BeeBotV2_atti(2,[1 3]),BeeBotV2_atti(3,[1 3]), 'ro', 'MarkerSize',5);
fig1_ARM24=plot3(gca,BeeBotV2_atti(1,[2 4]),BeeBotV2_atti(2,[2 4]),BeeBotV2_atti(3,[2 4]), 'bo', 'MarkerSize',5); 
fig1_payload=plot3(gca,BeeBotV2_atti(1,[5 6]),BeeBotV2_atti(2,[5 6]),BeeBotV2_atti(3,[5 6]), 'k', 'Linewidth',3);
fig1_shadow=plot3(gca,0,0,0,'xk','LineWidth',3);

hold(gca, 'off');
%% INIT. Data Fig.
fig2=figure('pos',[800 550 800 450]);
subplot(2,3,1)
title('phi [deg]');
grid on;
hold on;
subplot(2,3,2)
title('theta [deg]');
grid on;
hold on;
subplot(2,3,3)
title('psi [deg]');
grid on;
hold on;
subplot(2,3,4)
title('x [m]');
grid on;
hold on;
subplot(2,3,5)
title('y [m]');
grid on;
hold on;
subplot(2,3,6)
title('zdot [m/s]');
grid on;
hold on;


%%
commandSig(1)=10*D2R;
commandSig(2)= -10*D2R;
commandSig(3)=10*D2R;
commandSig(4)=1.0;

for i=1:simulationTime/0.01
    BeeBotV2.AttitudeCtrl(commandSig);
    BeeBotV2.UpdateState();
    
    BeeBotV2_state=BeeBotV2.GetState();
    
    %% 3D Plot
    figure(1)
    wHb = [RPY2Rot(BeeBotV2_state(4:6))' BeeBotV2_state(1:3); 0 0 0 1];
    BeeBotV2_world=wHb*BeeBotV2_body;
    BeeBotV2_atti=BeeBotV2_world(1:3, :);
    
    set(fig1_ARM13, ...
        'xData', BeeBotV2_atti(1,[1 3]), ...
        'yData', BeeBotV2_atti(2,[1 3]), ...
        'zData', BeeBotV2_atti(3,[1 3]));
    
    set(fig1_ARM24, ...
        'xData', BeeBotV2_atti(1,[2 4]), ...
        'yData', BeeBotV2_atti(2,[2 4]), ...
        'zData', BeeBotV2_atti(3,[2 4]));
    
    set(fig1_payload, ...
        'xData', BeeBotV2_atti(1,[5 6]), ...
        'yData', BeeBotV2_atti(2,[5 6]), ...
        'zData', BeeBotV2_atti(3,[5 6]));
    
    set(fig1_shadow, ...
        'xData', BeeBotV2_state(1), ...
        'yData', BeeBotV2_state(2), ...
        'zData', 0);
    
    
    figure(2)
    subplot(2,3,1)
        plot(i/100, BeeBotV2_state(4)*R2D, '.');
    subplot(2,3,2)
        plot(i/100, BeeBotV2_state(5)*R2D, '.');
    subplot(2,3,3)
        plot(i/100, BeeBotV2_state(6)*R2D, '.');
    subplot(2,3,4)
        plot(i/100, BeeBotV2_state(1), '.');
    subplot(2,3,5)
        plot(i/100, BeeBotV2_state(2), '.');
    subplot(2,3,6)
        plot(i/100, BeeBotV2_state(9), '.');
    
    drawnow;
    
    if (BeeBotV2_state(3) >=0)
        msgbox('Crashed!!','Error','error');
        break;
    end
end
