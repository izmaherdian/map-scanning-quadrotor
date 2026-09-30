%% PLOT_STATE_RESPONSE  Animated plots of position, Euler angles and control inputs U1-U4.
% Loads the saved simulation results in data/scanning_mode_results.mat.

% Load Data
load(fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'scanning_mode_results.mat'));

x = xData.Data(:,1);
x_d = xData.Data(:,2);
y = yData.Data(:,1);
y_d = yData.Data(:,2);
z = zData.Data(:,1);
z_d = zData.Data(:,2);
XYZs = [x y z];
XYZd = [x_d y_d z_d];

phi = phiData.Data(:,1);
phi_d = phiData.Data(:,2);
theta = thetaData.Data(:,1);
theta_d = thetaData.Data(:,2);
psi = psiData.Data(:,1);
psi_d = psiData.Data(:,2);
EulerAngles = [phi theta psi];
EulerAngled = [phi_d theta_d psi_d];

t = tData.Data(:,1);

xdot = xdotData.Data(:,1);
ydot = ydotData.Data(:,1);
zdot = zdotData.Data(:,1);

phidot = phidotData.Data(:,1);
thetadot = thetadotData.Data(:,1);
psidot = psidotData.Data(:,1);

UData = simOut.U_out;
U1 = UData.Data(:,1);
U2 = UData.Data(:,2);
U3 = UData.Data(:,3);
U4 = UData.Data(:,4);

figure;
set(gcf,'position',[0,0,1200,600])
sub1=subplot(4,3,1);
axis([0 waktu_simulasi min(XYZs(:,1))-2 max(XYZs(:,1))+2]);
ylabel('x (m)')
grid on
ani1=animatedline('Color','b','LineWidth',2);

subplot(4,3,4)
axis([0 waktu_simulasi min(XYZs(:,2))-2 max(XYZs(:,2))+2]); 
ylabel('y (m)')
grid on
ani2=animatedline('Color','g','LineWidth',2);

subplot(4,3,7)
axis([0 waktu_simulasi min(XYZs(:,3))-2 max(XYZs(:,3))+2]); 
ylabel('z (m)')
grid on
ani3=animatedline('Color','r','LineWidth',2);

subplot(4,3,2)
axis([0 waktu_simulasi min(EulerAngles(:,1))-0.01 max(EulerAngles(:,1))+0.01]); 
ylabel('Roll or phi (rad)')
grid on
ani4=animatedline('Color','b','LineWidth',2);

subplot(4,3,5)
axis([0 waktu_simulasi min(EulerAngles(:,2))-0.01 max(EulerAngles(:,2))+0.01]);
ylabel('Pitch or theta (rad)')
grid on
ani5=animatedline('Color','g','LineWidth',2);

subplot(4,3,8)
axis([0 waktu_simulasi min(EulerAngles(:,3))-0.001 max(EulerAngles(:,3))+0.001]);
ylabel('Yaw or psi (rad)')
grid on
ani6=animatedline('Color','r','LineWidth',2);

subplot(4,3,3)
axis([0 waktu_simulasi min(U1(:,1)) max(U1(:,1))]);
ylabel('U1')
grid on
ani7=animatedline('Color','black','LineWidth',2);

subplot(4,3,6)
axis([0 waktu_simulasi min(U2(:,1)) max(U2(:,1))]);
ylabel('U2')
grid on
ani8=animatedline('Color','black','LineWidth',2);

subplot(4,3,9)
axis([0 waktu_simulasi min(U3(:,1)) max(U3(:,1))]);
ylabel('U3')
grid on
ani9=animatedline('Color','black','LineWidth',2);

subplot(4,3,12)
axis([0 waktu_simulasi min(U4(:,1)) max(U4(:,1))]);
ylabel('U4')
grid on
ani10=animatedline('Color','black','LineWidth',2);

subplot(4,3,10)
axis off
text(0.1, 0.5, 'Time (s):', 'FontSize', 14, 'HorizontalAlignment', 'left');
TME = text(0.6, 0.5, num2str(0, '%.1f'), 'FontSize', 14, 'HorizontalAlignment', 'left');

subplot(4,3,11)
axis off
text(0.1, 0.5, 'Energy (J):', 'FontSize', 14, 'HorizontalAlignment', 'left');
ENG = text(0.6, 0.5, num2str(0, '%.2f'), 'FontSize', 14, 'HorizontalAlignment', 'left');

pause(1);

% Initialize current time and energy
current_time = 0;
Energy = 0;

% Update loop example (replace with actual simulation loop)
for t_idx = 1:length(t)
    % Update time and energy values
    current_time = t(t_idx);

    if t_idx == 1
        Power = (xdot(t_idx,1) - 0)^2 + (ydot(t_idx,1) - 0)^2 + (zdot(t_idx,1) - 0)^2 + ...
                (phidot(t_idx,1) - 0)^2 + (thetadot(t_idx,1) - 0)^2 + (psidot(t_idx,1) - 0)^2;
    else
        Power = (xdot(t_idx,1) - xdot(t_idx-1,1))^2 + (ydot(t_idx,1) - ydot(t_idx-1,1))^2 + ...
                (zdot(t_idx,1) - zdot(t_idx-1,1))^2 + (phidot(t_idx,1) - phidot(t_idx-1,1))^2 + ...
                (thetadot(t_idx,1) - thetadot(t_idx-1,1))^2 + (psidot(t_idx,1) - psidot(t_idx-1,1))^2;
    end

    % Accumulate energy
    if t_idx == 1
        dt = t(t_idx); % Untuk iterasi pertama
    else
        dt = t(t_idx) - t(t_idx-1); % Selisih waktu
    end
    Energy = Energy + Power * dt;


    % Update text on subplots
    set(TME, 'String', num2str(current_time, '%.1f'));
    set(ENG, 'String', num2str(Energy, '%.2f'));

    % Update animated lines
    addpoints(ani1, t(t_idx), XYZs(t_idx,1));    
    addpoints(ani2, t(t_idx), XYZs(t_idx,2));
    addpoints(ani3, t(t_idx), XYZs(t_idx,3));
    addpoints(ani4, t(t_idx), EulerAngles(t_idx,1));
    addpoints(ani5, t(t_idx), EulerAngles(t_idx,2));
    addpoints(ani6, t(t_idx), EulerAngles(t_idx,3));
    addpoints(ani7, t(t_idx), U1(t_idx,1));
    addpoints(ani8, t(t_idx), U2(t_idx,1));
    addpoints(ani9, t(t_idx), U3(t_idx,1));
    addpoints(ani10, t(t_idx), U4(t_idx,1));

    % Pause to simulate real-time updates
    pause(0.1);
end
