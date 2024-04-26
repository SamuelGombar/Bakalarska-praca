function fit = Fitness(pop)
    init;
    %% FIT FUNKCIA
    fit = 0;
    fit1 = 0;
    fit2 = 0;
    fit3 = 0;
    fit4 = 0;
    fit5 = 0;
    fit6 = 0;
    W1 = [pop(1:10); pop(11:20); pop(21:30); pop(31:40); pop(41:50)];
    B1 = pop(51:1:60);
    W2 = [pop(61:1:70); pop(71:1:80); pop(81:1:90); pop(91:1:100); pop(101:1:110);
    pop(111:1:120); pop(121:1:130); pop(131:1:140); pop(141:1:150); pop(151:1:160)];
    B2 = pop(161:1:170);
    W3 = [pop(171:1:180)' pop(181:1:190)'];
    
    v1.Position = [0 0 0];
%     checkpoints = zeros(1, ((sizeBoundary-3)/2));
    checkpoints = [];
    while advance(scenario)
        programStep = programStep + 1;
        
        dx = v*cos(theta + Beta(L, d));
        dy = v*sin(theta + Beta(L, d));
        thetad = (v/(L/2))*sin(Beta(L, d));
        theta = theta + thetad;
%     v1.Yaw = theta
        degrees = theta * (180/pi);
        v1.Yaw = degrees;
        v1.Position = v1.Position + [dx, dy, 0];       %1.175 - tretia tretina auta, teda predok
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
%             A = [xA yA];
%             B = [xB yB];
            n6 = (p4-p3)*(q3-yA)-(q4-q3)*(p3-xA);
            b6 = (p4-p3)*(yB-yA)-(q4-q3)*(xB-xA);
            c6 = (xB-xA)*(q3-yA)-(yB-yA)*(p3-xA);
            alfa = n6/b6;
            betta = c6/b6;
            if (alfa >= 0 && alfa <= 1) && (betta >= 0 && betta <= 1)
                isCollision = true;
                break        %POZOR SI PRIDAL BREAK
            end
    
            xA1 = v1.Position(1) + 1.175*cos(theta) - sin(theta);
            yA1 = v1.Position(2) + cos(theta) + 1.175*sin(theta);
            xB1 = v1.Position(1) + 1.175*cos(theta) + sin(theta);
            yB1 = v1.Position(2) - cos(theta) + 1.175*sin(theta);
%             D1 = [xA1 yA1];
%             E1 = [xB1 yB1];
            n7 = (p4-p3)*(q3-yA1)-(q4-q3)*(p3-xA1);
            b7 = (p4-p3)*(yB1-yA1)-(q4-q3)*(xB-xA1);
            c7 = (xB1-xA1)*(q3-yA1)-(yB1-yA1)*(p3-xA1);
            alfa1 = n7/b7;
            betta1 = c7/b7;
            if (alfa1 >= 0 && alfa1 <= 1) && (betta1 >= 0 && betta1 <= 1)
                isCollision = true;
                break       %POZOR SI PRIDAL BREAK
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
                %plot([angle1(1) angle2(1)], [angle1(2), angle2(2)], 'Color', 'blue');
                checkpoints(j) = 1;
            end
        end
        
        a1 = (x*W1)+B1;
        z1 = tanh(a1);
        a2 = (z1*W2)+B2;
        z2 = tanh(a2);
        a3 = z2*W3;
        z3 = tanh(a3);
        incrementd = z3(1)/8;

        incrementv = z3(2)/10;
        %% spracovanie otacania kolesa
%         if z3(1) > 0.05
%             incrementd = maxIncrementd;
%         elseif z3(1) < 0.05
%             incrementd = -maxIncrementd;
%         end

        
        %% spracovanie rychlosti
%         if z3(2) > 0.05
%             incrementv = maxIncrementv;
%         elseif z3(2) < 0.05
%             incrementv = -maxIncrementv;
%         end
        
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

        %% FIT & FINISH
        V(programStep) = v;
        if (isCollision) 
            break
        end
        if (programStep > 1000) 
            break
        end
        if ((v1.Position(1) >= 100) && (v1.Position(1) <= 104))
            if ((v1.Position(2) <= -75) && (v1.Position(2) >= -85)) 
                break
            end
        end
    end
    fit1 = fit + programStep;        %ulozit fit hodnoty jednotlive
%     fprintf("fit program steps: %f\n", fit1);
    fit2 = fit - (sum(V)*1000)/programStep; % 450
%     fprintf("fit rychlost: %f\n", fit2);
    fit3 = fit - 10*sum(checkpoints);
%     fprintf("fit checkpoints: %f\n", fit3);
%     fit4 = fit + (10000*sum(abs(Incrementd))/programStep);
%     fprintf("fit increments: %f\n", fit4);
    if (isCollision) 
            fit5 = fit + 10000;
    end
    if (programStep > 1000) 
            fit6 = fit + 10000;
    end
    fit = fit1 + fit2 + fit3 + fit4 + fit5 + fit6;
%     (10000*sum(Incrementd))/programStep
%     hodnota = 100*sum(Incrementd)
end