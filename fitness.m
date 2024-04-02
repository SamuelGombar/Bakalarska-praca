function fit = fitness(pop)
    init;
    %% FIT FUNKCIA
    fit = 0;
    W1 = [pop(1:10); pop(11:20); pop(21:30); pop(31:40); pop(41:50)];
    B1 = pop(51:1:60);
    W2 = [pop(61:1:70); pop(71:1:80); pop(81:1:90); pop(91:1:100); pop(101:1:110);
    pop(111:1:120); pop(121:1:130); pop(131:1:140); pop(141:1:150); pop(151:1:160)];
    B2 = pop(161:1:170);
    W3 = [pop(171:1:180)' pop(181:1:190)'];

    turningMax = 0.33;
    maxSpeed = 0.45;
    incrementd = 0;
    theta = 0;
    d = 0;
    v = 0;
    programStep = 0;
    isCollision = false;
    
    v1.Position = [0 0 0];
    while advance(scenario)
        programStep = programStep + 1;
    
        %v = 0.4;
        
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
        
        a1 = (x*W1)+B1;
        z1 = tanh(a1);
        a2 = (z1*W2)+B2;
        z2 = tanh(a2);
        a3 = z2*W3;
        z3 = tanh(a3);
        incrementd = z3(1)/13;
        d = d + incrementd;
        incrementv = z3(2)/10;
        v = v + incrementv;

        if d > turningMax
            d = turningMax;
        elseif d < -turningMax
            d = -turningMax;
        end

        if v > maxSpeed
            v = maxSpeed;
        elseif v < -maxSpeed
            v = -maxSpeed;
        end



%         if z3 < 0
%             v = 0.5*(-z3(2));
%         else
%             v = 0.5*z3(2);
%         end
        %% FIT & FINISH
        fit = fit - v*500;
%      stupnovita
        if (isCollision) 
            fit = fit + 100000 - 5*programStep;
            break
        end
        %priamo umerna
%         if (programStep > 300) 
%             fit = fit + programStep;
%         end
        %mrtva pokuta
        if (programStep > 1000) 
            fit = fit + 1000000;
            break
       end
        if ((v1.Position(1) >= 100) && (v1.Position(1) <= 104))
            if ((v1.Position(2) <= -75) && (v1.Position(2) >= -85)) 
                fit = fit + programStep;
                break
            end
        end
%         if ((v1.Position(1) <= -20) || (v1.Position(1) >= 120))
% %             fit = fit + 10000;
% %             break
%             fit = 100000 - programStep;
%         end
%             
%         if ((v1.Position(2) <= 20) || (v1.Position(2) >= -100))
% %             fit = fit + 10000;
% %             break
%             fit = 100000 - programStep;
%         end
    end 

end