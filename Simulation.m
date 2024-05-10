%% Clearing
clc;
close all;
fit_1 = 0;
fit_2 = 0;
fit_3 = 0;
for mapy = 1:3
    %% Environment & Map
    if mapy == 1
        map_original;
    elseif mapy == 2
        map_zigzag;
    else
        map_S;
    end
    
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
    
    %% Start & Finish
    if mapy == 1
        map_original_startfinish;
    elseif mapy == 2
        map_zigzag_startfinish;
    else
        map_S_startfinish;
    end
%     plot(outerBoundary(:,1), outerBoundary(:,2));
    
    %% Radars
    sensorLength = 20;
    sensorFov = pi/4;
    
    %% Car
    L = 4.7;
    m = 1000;
    d = 0;
    v = 0;
    
    turningMax = 0.65;
    maxSpeed = 0.5;
    
    dF = 50;
    
    %% Simulation
    programStep = 0;
    theta = 0;
    incrementd = 0;
    
    isCollision = false;
    
    V = [];
    Incrementd = [];
    
    fit = 0;
    fit1 = 0;
    fit2 = 0;
    fit3 = 0;
    fit4 = 0;
    fit5 = 0;
    fit6 = 0;
    [sizeBoundary, ~] = size(outerBoundary);
    checkpoints = zeros(1, (sizeBoundary-3)/2);
    
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
    
    W1 = bestW1;
    B1 = bestB1;
    W2 = bestW2;
    B2 = bestB2;
    W3 = bestW3;
    
    % W1 = W1_30;
    % B1 = B1_30;
    % W2 = W2_30;
    % B2 = B2_30;
    % W3 = W3_30;
    
    % W1 = W1_10;
    % B1 = B1_10;
    % W2 = W2_10;
    % B2 = B2_10;
    % W3 = W3_10;
    
    while advance(scenario)
        xlim(XLIMITS);
        ylim(YLIMITS);
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
        alpharray1 = [0 0 0 0 0 0 0 0 0 0];
        alpharray2 = [0 0 0 0 0 0 0 0 0 0];
        alpharray3 = [0 0 0 0 0 0 0 0 0 0];
        alpharray4 = [0 0 0 0 0 0 0 0 0 0];
        alpharray5 = [0 0 0 0 0 0 0 0 0 0];
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
                calpha1 = calpha1 + 1;
                alpharray1(calpha1) = 6*alpha(1)-3;

            end 
    
            n2 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
            b2 = (p4-p3)*(q6-q1)-(q4-q3)*(p6-p1);
            c2 = (p6-p1)*(q3-q1)-(q6-q1)*(p3-p1);
            alpha(2) = n2/b2;
            beta(2) = c2/b2;
            if (alpha(2) >= 0 && alpha(2) <= 1) && (beta(2) >= 0 && beta(2) <= 1)
                calpha2 = calpha2 + 1;
                alpharray2(calpha2) = 6*alpha(2)-3;

            end
    
            n3 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
            b3 = (p4-p3)*(q10-q1)-(q4-q3)*(p10-p1);
            c3 = (p10-p1)*(q3-q1)-(q10-q1)*(p3-p1);
            alpha(3) = n3/b3;
            beta(3) = c3/b3;
            if (alpha(3) >= 0 && alpha(3) <= 1) && (beta(3) >= 0 && beta(3) <= 1)
                calpha3 = calpha3 + 1;
                alpharray3(calpha3) = 6*alpha(3)-3;

            end
    
            n4 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
            b4 = (p4-p3)*(q14-q1)-(q4-q3)*(p14-p1);
            c4 = (p14-p1)*(q3-q1)-(q14-q1)*(p3-p1);
            alpha(4) = n4/b4;
            beta(4) = c4/b4;
            if (alpha(4) >= 0 && alpha(4) <= 1) && (beta(4) >= 0 && beta(4) <= 1)
                calpha4 = calpha4 + 1;
                alpharray4(calpha4) = 6*alpha(4)-3;

            end
    
            n5 = (p4-p3)*(q3-q1)-(q4-q3)*(p3-p1);
            b5 = (p4-p3)*(q18-q1)-(q4-q3)*(p18-p1);
            c5 = (p18-p1)*(q3-q1)-(q18-q1)*(p3-p1);
            alpha(5) = n5/b5;
            beta(5) = c5/b5;
            if (alpha(5) >= 0 && alpha(5) <= 1) && (beta(5) >= 0 && beta(5) <= 1)
                calpha5 = calpha5 + 1;
                alpharray5(calpha5) = 6*alpha(5)-3;

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
                disp("TU SOM NARAZIL POZDLZ NA DLZKU")
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
            b7 = (p4-p3)*(yB1-yA1)-(q4-q3)*(xB1-xA1);
            c7 = (xB1-xA1)*(q3-yA1)-(yB1-yA1)*(p3-xA1);
            alfa1 = n7/b7;
            betta1 = c7/b7;
            if (alfa1 >= 0 && alfa1 <= 1) && (betta1 >= 0 && betta1 <= 1)
                disp("TU SOM NARAZIL KOLMO NA DLZKU")
                isCollision = true;
                v1.Position
    %             alfa1
    %             betta1
                plot([D1(1) E1(1)], [D1(2), E1(2)], 'Color', 'green');
            end
    
        end
        x(1) = min(alpharray1);
        x(2) = min(alpharray2);
        x(3) = min(alpharray3);
        x(4) = min(alpharray4);
        x(5) = min(alpharray5);
        calpha1 = 0;
        calpha2 = 0;
        calpha3 = 0;
        calpha4 = 0;
        calpha5 = 0;
        alpharray1 = [0 0 0 0 0 0 0 0 0 0];
        alpharray2 = [0 0 0 0 0 0 0 0 0 0];
        alpharray3 = [0 0 0 0 0 0 0 0 0 0];
        alpharray4 = [0 0 0 0 0 0 0 0 0 0];
        alpharray5 = [0 0 0 0 0 0 0 0 0 0];
        
    
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
    
    %     incrementv = z3(2)/10;
        incrementv = z3(2)*dF/m;
    
        Incrementd(programStep) = abs(incrementd);
    
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
        
%         fit4 = fit4 + abs(d*2);
        %% FIT & FINISH
        V(programStep) = v;
        if (isCollision) 
                    disp("TU")
            break
        end
        if (programStep > 2000) 
            break
        end
    
        if ((v1.Position(1) >= XFINISH1) && (v1.Position(1) <= XFINISH2))
            if ((v1.Position(2) <= YFINISH1) && (v1.Position(2) >= YFINISH2)) 
                break
            end
        end
    end
    fit1 = fit + 2*programStep;        %ulozit fit hodnoty jednotlive
    fprintf("fit program steps: %f\n", fit1);
    fit2 = fit - (sum(V)*1000)/programStep; % 450
    fprintf("fit rychlost: %f\n", fit2);
    fit3 = fit - 50*sum(checkpoints);
    fprintf("fit checkpoints: %f\n", fit3);
    % fit4 = fit + (100*sum(abs(Incrementd))/programStep);
%     for ind = 1:length(Incrementd)-1
%             temp = temp + abs((Incrementd(ind+1) - Incrementd(ind)));
%     end
%     fit4 = 500*temp;
%     fprintf("fit increments: %f\n", fit4);
    if (isCollision) 
            fit5 = fit + 10000;
    end
    if (programStep > 1500) 
            fit6 = fit + 10000;
    end
    fprintf("fit naburanie: %f\n", fit5);
    fit = fit1 + fit2 + fit3 + fit4 + fit5 + fit6;
    if mapy == 1
        fit_1 = fit
    elseif mapy == 2
        fit_2 = fit
    else
        fit_3 = fit
    end
    if (isCollision)
        continue
    end
    finalfit = fit_1 + fit_2 + fit_3
end
