close all
% Load Data
load('DataLintasanDroneFix(ScanningMode).mat');

% Simulink 3D Animation with Moving Target
figure;
xlimit = [-10 panjang+20];
ylimit = [-lebar-20 10];
zlimit = [-5 25];
width = 900;
height = 900;
NewFigure(xlimit,ylimit,zlimit,-60,50,width,height);
pause(1);

AnimEuler(t, XYZs(1:length(t),:), EulerAngles(1:length(t),:), xlimit, ylimit, zlimit, XYZd);

function NewFigure(xlim,ylim,zlim,viewx,viewy,w,h)
    set(gca, 'XLim', xlim,'YLim',ylim,'ZLim',zlim);
    view(viewx,viewy)
    x0=10;
    y0=10;
    set(gcf,'position',[x0,y0,w,h])
    hold on;
    grid on;
end

function AnimEuler(t,XYZs,EulerAngles, xlimit, ylimit, zlimit, XYZ__d)
    curve = animatedline('LineWidth', 1.5, 'LineStyle', ':', 'Color', 'b'); % Actual path
    curve_d = animatedline('LineWidth', 1.2, 'LineStyle', '--', 'Color', 'r'); % Desired path

    % Add legend only for Actual and Desired paths (fix legend issue)
    lgd = legend([curve, curve_d], {'Actual Path Quadrotor', 'Desired Path Quadrotor'}, ...
        'Location', 'northeast', 'FontSize', 10);

    % Fix legend visibility to exclude new elements
    set(lgd, 'AutoUpdate', 'off');
    
    t_section = 0;
    m_draw = [[1 0 0];[0 cos(pi) sin(pi)];[0 -sin(pi) cos(pi)]];
    
    current_time = 0;
    
    TME = text(xlimit(2)+1, ylimit(2)-7, zlimit(2)-10, ...
        sprintf('Time: %.1f', current_time), ...
        'FontSize', 18, 'HorizontalAlignment', 'left', 'BackgroundColor', 'white');

    % Do Animation
    for i = 1:length(t)-1
        if abs(t(i) - t_section) < 0.0001
            Euler = EulerAngles(i,:);
            XYZ = XYZs(i,:);
            XYZd = XYZ__d(i,:);
            
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O;
            
            scalesumbu = 3;  % Skala panjang sumbu
            Xb = transpose(XYZ) + scalesumbu * O_I(:,1);  
            Yb = transpose(XYZ) + scalesumbu * O_I(:,2);
            Zb = transpose(XYZ) + scalesumbu * O_I(:,3);

            
            scaledrone = 3; % Sesuaikan nilai ini untuk mengatur panjang lengan drone
            FR = transpose(XYZ) + scaledrone*O_I(:,1) + scaledrone*O_I(:,2);
            FL = transpose(XYZ) + scaledrone*O_I(:,1) - scaledrone*O_I(:,2);
            BR = transpose(XYZ) - scaledrone*O_I(:,1) + scaledrone*O_I(:,2);
            BL = transpose(XYZ) - scaledrone*O_I(:,1) - scaledrone*O_I(:,2);

            
            % NED coordinate to Draw coordinate
            Xb = transpose(m_draw*Xb);
            Yb = transpose(m_draw*Yb);
            Zb = transpose(m_draw*Zb);
            
            FR = transpose(m_draw*FR);
            FL = transpose(m_draw*FL);
            BR = transpose(m_draw*BR);
            BL = transpose(m_draw*BL);
            
            XYZ = transpose(m_draw*transpose(XYZ));
            XYZ_d = transpose(m_draw * transpose(XYZd));
            
            % Draw
            addpoints(curve, XYZ(1), XYZ(2),XYZ(3));
            addpoints(curve_d, XYZ_d(1), XYZ_d(2), XYZ_d(3)); % Desired path
            pts = [XYZ;Xb];
            line1 = plot3(pts(:,1), pts(:,2), pts(:,3),'b','LineWidth',1);
            pts = [XYZ;Yb];
            line2 = plot3(pts(:,1), pts(:,2), pts(:,3),'g','LineWidth',1);
            pts = [XYZ;Zb];
            line3 = plot3(pts(:,1), pts(:,2), pts(:,3),'r','LineWidth',1);
            
            pts = [XYZ;FR];
            frame1 = plot3(pts(:,1), pts(:,2), pts(:,3),'black','LineWidth',2.5);
            pts = [XYZ;FL];
            frame2 = plot3(pts(:,1), pts(:,2), pts(:,3),'black','LineWidth',2.5);
            pts = [XYZ;BR];
            frame3 = plot3(pts(:,1), pts(:,2), pts(:,3),'black','LineWidth',2.5);
            pts = [XYZ;BL];
            frame4 = plot3(pts(:,1), pts(:,2), pts(:,3),'black','LineWidth',2.5); 
            
            % Draw a marker at the desired position (XYZ_d)
            marker_d = plot3(XYZ_d(1), XYZ_d(2), XYZ_d(3), 'o', 'MarkerSize', 2.5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
 
            TME.String = sprintf('Time: %.1f', t(i));

            drawnow
            pause(0.05)
            

            delete(line1)
            delete(line2)
            delete(line3)       
            delete(frame1)
            delete(frame2)
            delete(frame3)
            delete(frame4)
            delete(marker_d)
            
            t_section = t_section + 0.1;
        end
    end
end

function m = matrixB2I(phi,theta,psi)
    R_ItoV1= [[cos(psi) sin(psi) 0];[-sin(psi) cos(psi) 0];[0 0 1]];
    R_V1toV2 = [[cos(theta) 0 -sin(theta)];[0 1 0];[sin(theta) 0 cos(theta)]];
    R_V2toB = [[1 0 0];[0 cos(phi) sin(phi)];[0 -sin(phi) cos(phi)]];
    m = R_V2toB*R_V1toV2*R_ItoV1;
    m = transpose(m);
end

function line = drawline(p1,p2,color,width)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1;
    pt2 = pt1 + transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',width);
end