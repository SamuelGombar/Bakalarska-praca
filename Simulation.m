%% Clearing
clc;
close all;

%% Environment & Map
scenario = drivingScenario;
Map1;

%% Actor
v1 = vehicle(scenario, 'ClassID',1', 'Position',[0 0 0], 'Velocity',[0.1 -0.5 0], 'Yaw', 0);
v1.RearOverhang = 2.35;

%% Figures & Plot
hFigure = figure;
hFigure.Position = [100 100 900 680];

hPanel1 = uipanel(hFigure,'Units','Normalized','Title','Scenario Plot');

hAxes1 = axes('Parent',hPanel1);

plot(scenario, 'Parent', hAxes1);
hold on;

%% Start
plot([-4 -4 0 0 -4], [5 -5 -5 5 5], 'Color', 'black');
fill([-4 -4 0 0 -4], [5 -5 -5 5 5], 'green');

%% Finish
for j = 0:2:2
    flop = true;
    if j == 2 
        flop = false;
    end
    xfinish = [100+j 100+j 102+j 102+j 100+j];
    for i = 0:2.5:7.5
        yfinish = [-75-i -77.5-i -77.5-i -75-i -75-i];
        hplot = plot(xfinish, yfinish);
        hplot.Color = 'black';
        if flop == true
            fill(xfinish, yfinish, 'white');
            flop = false;
        else
            fill(xfinish, yfinish, 'black');
            flop = true;
        end
    end
end

%% Radars
sensorLength = 20;
sensorFov = 5*pi/8; %5*pi/6;

%% Simulation
L = 4.7;

incrementd = 0;
theta = 0;
d = 0;
programStep = 0;
collision = 0;
isCollision = false;
turningMax = 0.4;
maxSpeed = 0.5;
fit = 0;
v = 0;
counter = 0;
[sizeBoundary, ~] = size(outerBoundary);
checkpoints = zeros(1, (sizeBoundary-3)/2);
maxIncrementd = 0.08;
maxIncrementv = 0.05;

% W1 = bestW1;
% B1 = bestB1;
% W2 = bestW2;
% B2 = bestB2;
% W3 = bestW3;

W1 = W1_50;
B1 = B1_50;
W2 = W2_50;
B2 = B2_50;
W3 = W3_50;

% W1 = W1_15;
% B1 = B1_15;
% W2 = W2_15;
% B2 = B2_15;
% W3 = W3_15;

while advance(scenario)
    xlim([-5 106]);
    ylim([-95 15]);
    programStep = programStep + 1;

    dx = v*cos(theta + Beta(L, d));
    dy = v*sin(theta + Beta(L, d));
    thetad = (v/(L/2))*sin(Beta(L, d));
    theta = theta + thetad;
    degrees = theta * (180/pi);
    v1.Yaw = degrees;
    v1.Position = v1.Position + [dx, dy, 0];
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
    for i = 1:size(outerBoundary)
        %% Sensors
        p1 = frontAxle(1);
        q1 = frontAxle(2);
        p2 = tipOfSensor1(1);
        q2 = tipOfSensor1(2);
        p3 = outerBoundary(i,1);
        q3 = outerBoundary(i,2);

        if (i == size(outerBoundary,1))
            p4 = outerBoundary(1,1);
            q4 = outerBoundary(1,2);
        else
            p4 = outerBoundary(i+1,1);
            q4 = outerBoundary(i+1,2);
        end

        p6 = tipOfSensor2(1);
        q6 = tipOfSensor2(2);

        p10 = tipOfSensor3(1);
        q10 = tipOfSensor3(2);

        p14 = tipOfSensor4(1);
        q14 = tipOfSensor4(2);

        p18 = tipOfSensor5(1);
        q18 = tipOfSensor5(2);

        n1 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b1 = (p4-p3)*(q2-q1)-(q4-q3)*(p2-p1);
        c1 = (p2-p1)*(q3-q1)-(q2-q1)*(p3-p1);
        alpha(1) = n1/b1;
        beta(1) = c1/b1;
        if (alpha(1) >= 0 && alpha(1) <= 1) && (beta(1) >= 0 && beta(1) <= 1)
            x(1) = alpha(1)*2-1;
        end 

        n2 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b2 = (p4-p3)*(q6-q1)-(q4-q3)*(p6-p1);
        c2 = (p6-p1)*(q3-q1)-(q6-q1)*(p3-p1);
        alpha(2) = n2/b2;
        beta(2) = c2/b2;
        if (alpha(2) >= 0 && alpha(2) <= 1) && (beta(2) >= 0 && beta(2) <= 1)
            x(2) = alpha(2)*2-1;
        end

        n3 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b3 = (p4-p3)*(q10-q1)-(q4-q3)*(p10-p1);
        c3 = (p10-p1)*(q3-q1)-(q10-q1)*(p3-p1);
        alpha(3) = n3/b3;
        beta(3) = c3/b3;
        if (alpha(3) >= 0 && alpha(3) <= 1) && (beta(3) >= 0 && beta(3) <= 1)
            x(3) = alpha(3)*2-1;
        end

        n4 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b4 = (p4-p3)*(q14-q1)-(q4-q3)*(p14-p1);
        c4 = (p14-p1)*(q3-q1)-(q14-q1)*(p3-p1);
        alpha(4) = n4/b4;
        beta(4) = c4/b4;
        if (alpha(4) >= 0 && alpha(4) <= 1) && (beta(4) >= 0 && beta(4) <= 1)
            x(4) = alpha(4)*2-1;
        end

        n5 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
        b5 = (p4-p3)*(q18-q1)-(q4-q3)*(p18-p1);
        c5 = (p18-p1)*(q3-q1)-(q18-q1)*(p3-p1);
        alpha(5) = n5/b5;
        beta(5) = c5/b5;
        if (alpha(5) >= 0 && alpha(5) <= 1) && (beta(5) >= 0 && beta(5) <= 1)
            x(5) = alpha(5)*2-1;
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
            isCollision = true;
            plot([A(1) B(1)], [A(2), B(2)], 'Color', 'blue');
        end
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
            isCollision = true;
            plot([D1(1) E1(1)], [D1(2), E1(2)], 'Color', 'green');
        end

    end

    %checkpoints
    [sizeBoundary, ~] = size(outerBoundary); 
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
        angle1 = [pcheck1 qcheck1];
        angle2 = [pcheck2 qcheck2];
        if (alfa2 >= 0 && alfa2 <= 1) && (betta2 >= 0 && betta2 <= 1)
            plot([angle1(1) angle2(1)], [angle1(2), angle2(2)], 'Color', 'blue');
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

    a1 = (x*W1)+B1;
    z1 = tanh(a1);
    a2 = (z1*W2)+B2;
    z2 = tanh(a2);
    a3 = z2*W3;
    z3 = tanh(a3);
    incrementd = z3(1)/10;

    incrementv = z3(2)/10;
    %% spracovanie otacania kolesa
%     if z3(1) > 0
%         incrementd = maxIncrementd;
%     elseif z3(1) < -0
%         incrementd = -maxIncrementd;
%     end
%     
%     %% spracovanie rychlosti
%     if z3(2) > 0
%         incrementv = maxIncrementv;
%     elseif z3(2) < -0
%         incrementv = -maxIncrementv;
%     end

    counter = counter + 1;
    Incrementd(counter) = abs(incrementd);
    
    %% ochrana pred prekrocenim maxu
    if (d + incrementd) > turningMax
        incrementd = 0;
    elseif (d + incrementd) < -turningMax
        incrementd = 0;
    end
    if (v + incrementv) > maxSpeed
        incrementv = 0;
    elseif (v + incrementv) < -maxSpeed
        incrementv = 0;
    end

    d = d + incrementd;
    v = v + incrementv;


 %% FIT & FINISH
    V(counter) = v;

    if (isCollision) 
        fit = fit + 10000;
        break
    end
    if (programStep > 1000) 
        fit = fit + 10000;
        break
    end
    if ((v1.Position(1) >= 100) && (v1.Position(1) <= 104))
        if ((v1.Position(2) <= -75) && (v1.Position(2) >= -85)) 
            break
        end
    end
end
fit = fit + programStep;
fit = fit - (sum(V)*1000)/programStep; % 450
fit = fit - 100*sum(checkpoints)
fit = fit + (100*sum(abs(Incrementd)))/programStep;
% (1000*sum(Incrementd))/programStep
% fit = fit + 10*sum(Incrementd)
% hodnota = 100*sum(Incrementd);
