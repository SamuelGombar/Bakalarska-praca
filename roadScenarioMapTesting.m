%% Clearing
close all;
clear;
clc;

%% Environment & road creation
% roadCenters = [0 0; 80 0; 81 0; 81 -40; 80 -40; 70 -37; 60 -43; 50 -37; 40 -43; 30 -37; 21 -40; 20 -40; 20 -80; 21 -80; 100 -80];
map_zigzag;
%% Actor
v1 = vehicle(scenario, 'ClassID',1', 'Position',[0 0 0], 'Velocity',[0.1 -0.5 0], 'Yaw',0);
v1.RearOverhang = 2.35;

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
plot(outerBoundary(:,1), outerBoundary(:,2));

%% Start & Finish
map_zigzag_startfinish;

%% Radars
sensorLength = 20;
sensorFov = 5*pi/10;

%% Simulation

set(hFigure, 'KeyPressFcn', @kbInput1);

L = 4.7;
[sizeBoundary, ~] = size(outerBoundary);
checkpoints = zeros(1, ((sizeBoundary-3)/2));

theta = 0;
global d;
global v;
global Finc;
global m;
m = 500;
Fmax = 200;
Finc = 50;
d = 0;
v = 0;
dF = Finc;
programStep = 0;
%isCollision = false;

calpha1 = 0;
calpha2 = 0;
calpha3 = 0;
calpha4 = 0;
calpha5 = 0;
alpharray1 = [];
alpharray2 = [];
alpharray3 = [];
alpharray4 = [];
alpharray5 = [];
while advance(scenario)
    programStep = programStep + 1;
    xlim(XLIMITS);
    ylim(YLIMITS);
%     R = L / tan(d);       %zmena sin, cos, degrees v uhloch sensorov,
%     sensor fov pridat, vzdialenost frontaxle 2.525
%     w = v / R;
    %theta = theta + d;
%     theta = theta + v*tand(d)/L;
%     dx = v*cos(theta);
%     dy = v*sin(theta);

    Flong = (dF/m)*cos(Beta(L,d));
    Flat = (dF/m)*sin(Beta(L,d));
    dx = v*cos(theta + Beta(L, d));
    dy = v*sin(theta + Beta(L, d));
    thetad = (v/(L/2))*sin(Beta(L, d));
    theta = theta + thetad;
%     v1.Yaw = theta
    degrees = theta * (180/pi);
        lateralspeed = v*sin(Beta(L,d));
    longitudinalspeed = v*cos(Beta(L,d));
    v1.Yaw = degrees;
    v1.Position = v1.Position + [dx, dy, 0];    %1 + 2.525 = 3.525 = tristvrtina auta vpredu //zastarale, asi vymazat
    frontAxle = [v1.Position(1) + 1.175*cos(theta) v1.Position(2) + 1.175*sin(theta)]; %vektor obsahujuci x y front axle
    tipOfSensor1 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(sensorFov/2+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(sensorFov/2+theta)];
    tipOfSensor2 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(sensorFov/4+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(sensorFov/4+theta)];
    tipOfSensor3 = [v1.Position(1) + 1.175*cos(theta) + sensorLength*cos(-sensorFov/4+theta)...
        v1.Position(2) + 1.175*sin(theta) + sensorLength*sin(-sensorFov/4+theta)];
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
            calpha1 = calpha1 + 1;
            alpharray1(calpha1) = 6*alpha(1)-3;
            x(1) = min(alpharray1);
        end 

        n2 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b2 = (p4-p3)*(q6-q1)-(q4-q3)*(p6-p1);
        c2 = (p6-p1)*(q3-q1)-(q6-q1)*(p3-p1);
        alpha(2) = n2/b2;
        beta(2) = c2/b2;
        if (alpha(2) >= 0 && alpha(2) <= 1) && (beta(2) >= 0 && beta(2) <= 1)
            calpha2 = calpha2 + 1;
            alpharray2(calpha2) = 6*alpha(2)-3;
            x(2) = min(alpharray2);
        end

        n3 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b3 = (p4-p3)*(q10-q1)-(q4-q3)*(p10-p1);
        c3 = (p10-p1)*(q3-q1)-(q10-q1)*(p3-p1);
        alpha(3) = n3/b3;
        beta(3) = c3/b3;
        if (alpha(3) >= 0 && alpha(3) <= 1) && (beta(3) >= 0 && beta(3) <= 1)
            calpha3 = calpha3 + 1;
            alpharray3(calpha3) = 6*alpha(3)-3;
            x(3) = min(alpharray3);
        end

        n4 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b4 = (p4-p3)*(q14-q1)-(q4-q3)*(p14-p1);
        c4 = (p14-p1)*(q3-q1)-(q14-q1)*(p3-p1);
        alpha(4) = n4/b4;
        beta(4) = c4/b4;
        if (alpha(4) >= 0 && alpha(4) <= 1) && (beta(4) >= 0 && beta(4) <= 1)
            calpha4 = calpha4 + 1;
            alpharray4(calpha4) = 6*alpha(4)-3;
            x(4) = min(alpharray4);
        end

        n5 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b5 = (p4-p3)*(q18-q1)-(q4-q3)*(p18-p1);
        c5 = (p18-p1)*(q3-q1)-(q18-q1)*(p3-p1);
        alpha(5) = n5/b5;
        beta(5) = c5/b5;
        if (alpha(5) >= 0 && alpha(5) <= 1) && (beta(5) >= 0 && beta(5) <= 1)
            calpha5 = calpha5 + 1;
            alpharray5(calpha5) = 6*alpha(5)-3;
            x(5) = min(alpharray5);
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
    end
    calpha1 = 0;
    calpha2 = 0;
    calpha3 = 0;
    calpha4 = 0;
    calpha5 = 0;
    alpharray1 = 0;
    alpharray2 = 0;
    alpharray3 = 0;
    alpharray4 = 0;
    alpharray5 = 0;

    %checkpoints
    for j = 1:(sizeBoundary-3)/2
        pcheck1 = outerBoundary(j+3,1);
        qcheck1 = outerBoundary(j+3,2);
        pcheck2 = outerBoundary(sizeBoundary-j+1,1);
        qcheck2 = outerBoundary(sizeBoundary-j+1,2);
        n8 = (pcheck2-pcheck1)*(qcheck1-yA)-(qcheck2-qcheck1)*(pcheck1-xA);
        b8 = (pcheck2-pcheck1)*(yB-yA)-(qcheck2-qcheck1)*(xB-xA);
        c8 = (xB-xA)*(qcheck1-yA)-(yB-yA)*(pcheck1-xA);
        alfa2 = n8/b8;
        betta2 = c8/b8;
        blah1 = [pcheck1 qcheck1];
        blah2 = [pcheck2 qcheck2];
        if (alfa2 >= 0 && alfa2 <= 1) && (betta2 >= 0 && betta2 <= 1)
            plot([blah1(1) blah2(1)], [blah1(2), blah2(2)], 'Color', 'blue');
            checkpoints(j) = 1;
        end
    end

    h1 = plot([frontAxle(1) tipOfSensor1(1)], [frontAxle(2) tipOfSensor1(2)], 'LineWidth', 0.5, 'Color', 'red');
    h2 = plot([frontAxle(1) tipOfSensor2(1)], [frontAxle(2) tipOfSensor2(2)], 'LineWidth', 0.5, 'Color', 'red');
    h3 = plot([frontAxle(1) tipOfSensor3(1)], [frontAxle(2) tipOfSensor3(2)], 'LineWidth', 0.5, 'Color', 'red');
    h4 = plot([frontAxle(1) tipOfSensor4(1)], [frontAxle(2) tipOfSensor4(2)], 'LineWidth', 0.5, 'Color', 'red');
    h5 = plot([frontAxle(1) tipOfSensor5(1)], [frontAxle(2) tipOfSensor5(2)], 'LineWidth', 0.5, 'Color', 'red');
    pause(0.02);
    delete(h1);
    delete(h2);
    delete(h3);
    delete(h4);
    delete(h5);

    x

    if ((v1.Position(1) >= XFINISH1) && (v1.Position(1) <= XFINISH2))
        if ((v1.Position(2) <= YFINISH1) && (v1.Position(2) >= YFINISH2)) 
            break
        end
    end
end

function kbInput1(~, event)
    global d;
    global Finc;
    global m;
    global v;
    switch event.Key
        case 'a'
            d = d + 0.1; %bolo 0.5
%               d = d + 
        case 'd'
            d = d - 0.1;
        case 'w'
            v = v + Finc/m;
        case 's'
            v = v - Finc/m;
    end
end

