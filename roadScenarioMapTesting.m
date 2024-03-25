%% Clearing
close all;
clear;
clc;

%% Bicycle kinematics

%% Environment & road creation
scenario = drivingScenario;
roadCenters = [0 0; 80 0; 81 0; 81 -40; 80 -40; 70 -37; 60 -43; 50 -37; 40 -43; 30 -37; 21 -40; 20 -40; 20 -80; 21 -80; 100 -80];
%roadCenters = [20 30; 79 30; 80 30; 100 0; 80 -30; 79 -30; 20 -30; 20 30]; %circle map
roadWidth = 10;
road(scenario, roadCenters, roadWidth);

%% Actor
%global v1;
v1 = vehicle(scenario, 'ClassID',1', 'Position',[0 0 0], 'Velocity',[0.1 -0.5 0], 'Yaw',0);
v1.RearOverhang = 2.35;

%% Actor trajectory
%smoothTrajectory(v1, roadCenters, 30);

%% Boundaries
rb = roadBoundaries(scenario);
% innerBoundary = rb{1};
% outerBoundary = rb{2};
outerBoundary = rb{1};
outerBoundary(181,1) = 150;
outerBoundary(182,1) = 150;
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;
 
%% Figures & plot
%plot(scenario);
%set(gcf, 'Name', 'Road Scenario');
%xlim([15 110]);
%ylim([-40 40]);
hFigure = figure;
%hFigure.Position(3) = 900;
hFigure.Position = [100 100 900 680];

hPanel1 = uipanel(hFigure,'Units','Normalized','Title','Scenario Plot');
%hPanel1 = uipanel(hFigure,'Units','Normalized','Position',[0 0 1/2 1],'Title','Scenario Plot');
%hPanel2 = uipanel(hFigure,'Units','Normalized','Position',[1/2 0 1/2 1],'Title','Chase Plot');

hAxes1 = axes('Parent',hPanel1);
%hAxes2 = axes('Parent',hPanel2);

plot(scenario, 'Parent', hAxes1);
%chasePlot(v1, 'Parent', hAxes2);
hold on;

%boundary plot
%plot(innerBoundary(:,1), innerBoundary(:,2));
%plot(outerBoundary(:,1), outerBoundary(:,2));

%% Radars
sensorLength = 15;
sensorFov = 5*pi/6;

%% Simulation
%chasePlot(v1);
%plot3(boundary(:,1),boundary(:,2),boundary(:,3),'g');
%axis equal;
%scenario.StopTime = 10;
%fps = 60;
%pauseTime = 1/fps;

set(hFigure, 'KeyPressFcn', @kbInput1);

L = 4.7;

theta = 0;
global d;
d = 0;
%isCollision = false;
while advance(scenario)
    v = 0.4;
%     R = L / tan(d);       %zmena sin, cos, degrees v uhloch sensorov,
%     sensor fov pridat, vzdialenost frontaxle 2.525
%     w = v / R;
    %theta = theta + d;
%     theta = theta + v*tand(d)/L;
%     dx = v*cos(theta);
%     dy = v*sin(theta);
    dx = v*cos(theta + Beta(L, d));
    dy = v*sin(theta + Beta(L, d));
    thetad = (v/(L/2))*sin(Beta(L, d));
    theta = theta + thetad;
%     v1.Yaw = theta
    degrees = theta * (180/pi);
    v1.Yaw = degrees;
    v1.Position = v1.Position + [dx, dy, 0];    %1 + 2.525 = 3.525 = tristvrtina auta vpredu //zastarale, asi vymazat
    frontAxle = [v1.Position(1) + 1.175*cos(theta) v1.Position(2) + 1.175*sin(theta)]; %vektor obsahujuci x y front axle
    tipOfSensor1 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(sensorFov/2+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(sensorFov/2+theta)];
    tipOfSensor2 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(sensorFov/8+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(sensorFov/8+theta)];
    tipOfSensor3 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(-sensorFov/8+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(-sensorFov/8+theta)];
    tipOfSensor4 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(-sensorFov/2+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(-sensorFov/2+theta)];
    tipOfSensor5 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(theta)];
    

    alpha = [0 0 0 0 0];
    beta = [0 0 0 0 0];
    x = [0 0 0 0 0];
    collision = 0;
    for i = 1:size(outerBoundary)
        %% Sensors
        p1 = frontAxle(1);
        q1 = frontAxle(2);
        p2 = tipOfSensor1(1);
        q2 = tipOfSensor1(2);
        p3 = outerBoundary(i,1);
        q3 = outerBoundary(i,2);

        p6 = tipOfSensor2(1);
        q6 = tipOfSensor2(2);

        p10 = tipOfSensor3(1);
        q10 = tipOfSensor3(2);

        p14 = tipOfSensor4(1);
        q14 = tipOfSensor4(2);

        p18 = tipOfSensor5(1);
        q18 = tipOfSensor5(2);

        if (i == size(outerBoundary,1))
            p4 = outerBoundary(1,1);
            q4 = outerBoundary(1,2);
        else
            p4 = outerBoundary(i+1,1);
            q4 = outerBoundary(i+1,2);
        end

        n1 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b1 = (p4-p3)*(q2-q1)-(q4-q3)*(p2-p1);
        c1 = (p2-p1)*(q3-q1)-(q2-q1)*(p3-p1);
        alpha(1) = n1/b1;
        beta(1) = c1/b1;
        if (alpha(1) >= 0 && alpha(1) <= 1) && (beta(1) >= 0 && beta(1) <= 1)
            x(1) = alpha(1);
        end 

        n2 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b2 = (p4-p3)*(q6-q1)-(q4-q3)*(p6-p1);
        c2 = (p6-p1)*(q3-q1)-(q6-q1)*(p3-p1);
        alpha(2) = n2/b2;
        beta(2) = c2/b2;
        if (alpha(2) >= 0 && alpha(2) <= 1) && (beta(2) >= 0 && beta(2) <= 1)
            x(2) = alpha(2);
        end

        n3 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b3 = (p4-p3)*(q10-q1)-(q4-q3)*(p10-p1);
        c3 = (p10-p1)*(q3-q1)-(q10-q1)*(p3-p1);
        alpha(3) = n3/b3;
        beta(3) = c3/b3;
        if (alpha(3) >= 0 && alpha(3) <= 1) && (beta(3) >= 0 && beta(3) <= 1)
            x(3) = alpha(3);
        end

        n4 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b4 = (p4-p3)*(q14-q1)-(q4-q3)*(p14-p1);
        c4 = (p14-p1)*(q3-q1)-(q14-q1)*(p3-p1);
        alpha(4) = n4/b4;
        beta(4) = c4/b4;
        if (alpha(4) >= 0 && alpha(4) <= 1) && (beta(4) >= 0 && beta(4) <= 1)
            x(4) = alpha(4);
        end

        n5 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b5 = (p4-p3)*(q18-q1)-(q4-q3)*(p18-p1);
        c5 = (p18-p1)*(q3-q1)-(q18-q1)*(p3-p1);
        alpha(5) = n5/b5;
        beta(5) = c5/b5;
        if (alpha(5) >= 0 && alpha(5) <= 1) && (beta(5) >= 0 && beta(5) <= 1)
            x(5) = alpha(5);
        end

        %collision
        xA = v1.Position(1) - cos(theta);
        yA = v1.Position(2) - sin(theta);
        xB = v1.Position(1) + 1.175*cos(theta);
        yB = v1.Position(2) + 1.175*sin(theta);
        A = [xA yA];
        B = [xB yB];
        n6 = (p4-p3)*(q3-yA)-(q4-q3)*(p3-xA);
        b6 = (p4-p3)*(yB-yA)-(q4-q3)*(xB-xA);
        c6 = (xB-xA)*(q3-yA)-(yB-yA)*(p3-xA);
        alfa = n6/b6;
        betta = c6/b6;
        if (alfa >= 0 && alfa <= 1) && (betta >= 0 && betta <= 1)
            %isCollision = true;
            plot([A(1) B(1)], [A(2), B(2)], 'Color', 'blue');
        end
        %plot([A(1) B(1)], [A(2), B(2)], 'Color', 'green');

        xA1 = v1.Position(1) + 1.175*cos(theta) - sin(theta);
        yA1 = v1.Position(2) + cos(theta) + 1.175*sin(theta);
        xB1 = v1.Position(1) + 1.175*cos(theta) + sin(theta);
        yB1 = v1.Position(2) - cos(theta) + 1.175*sin(theta);
        D1 = [xA1 yA1];
        E1 = [xB1 yB1];
        n7 = (p4-p3)*(q3-yA1)-(q4-q3)*(p3-xA1);
        b7 = (p4-p3)*(yB1-yA1)-(q4-q3)*(xB-xA1);
        c7 = (xB1-xA1)*(q3-yA1)-(yB1-yA1)*(p3-xA1);
        alfa1 = n7/b7;
        betta1 = c7/b7;
        if (alfa1 >= 0 && alfa1 <= 1) && (betta1 >= 0 && betta1 <= 1)
            plot([D1(1) E1(1)], [D1(2), E1(2)], 'Color', 'green');
            %isCollision = true;
        end
        %plot([A1(1) B1(1)], [A1(2), B1(2)], 'Color', 'green');
    end

    h1 = plot([frontAxle(1) tipOfSensor1(1)], [frontAxle(2) tipOfSensor1(2)], 'LineWidth', 0.5, 'Color', 'red');
    h2 = plot([frontAxle(1) tipOfSensor2(1)], [frontAxle(2) tipOfSensor2(2)], 'LineWidth', 0.5, 'Color', 'red');
    h3 = plot([frontAxle(1) tipOfSensor3(1)], [frontAxle(2) tipOfSensor3(2)], 'LineWidth', 0.5, 'Color', 'red');
    h4 = plot([frontAxle(1) tipOfSensor4(1)], [frontAxle(2) tipOfSensor4(2)], 'LineWidth', 0.5, 'Color', 'red');
    h5 = plot([frontAxle(1) tipOfSensor5(1)], [frontAxle(2) tipOfSensor5(2)], 'LineWidth', 0.5, 'Color', 'red');
    %updatePlots(scenario);
    pause(0.02);
    delete(h1);
    delete(h2);
    delete(h3);
    delete(h4);
    delete(h5);
    %delete(stena1);
    %disp(collision);
    %disp(x);

    x = alpha;

%     W1 = zeros(5,10);
%     W2 = zeros(10,2);
%     B1 = zeros(1,10);
% 
%     n1 = (x*W1)+B1;
%     z1 = tanh(n1);
%     n2 = z1*W2;
%     z2 = tanh(n2);
% if (isCollision) 
%         return
%     end
end

function kbInput1(~, event)
    global d;
    switch event.Key
        case 'a'
            d = d + 0.1; %bolo 0.5
        case 'd'
            d = d - 0.1;
    end
end

