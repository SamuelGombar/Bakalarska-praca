%% Clearing
close all;
clear;
clc;
% load LatestSpecimen.mat;
exists = false;

maxGen = 50;
popSize = 50;
% numberOfGenes = 190;
numberOfGenes = 230;
% numberOfGenes = 80;
paramInterval = [ones(1,numberOfGenes)*-3; ones(1,numberOfGenes)*3];

    %% Island 1
    fit1 = zeros(1, popSize);
    pop1 = zeros(popSize, numberOfGenes);
    
    %% Island 2
    fit2 = zeros(1, popSize);
    pop2 = zeros(popSize, numberOfGenes);
    
    if exist('bestPop', 'var') == 1
        exists = true;
        pop1 = bestPop;
        fit1 = bestFit;
        pop2 = bestPop;
        fit2 = bestFit;
        pop3 = bestPop;
        fit3 = bestFit;
    else
        %% Island 3
        fit3 = zeros(1, popSize);
        pop3 = zeros(popSize, numberOfGenes);
    end

for gen = 1:maxGen
    disp(gen);
    if mod(gen, 20) == 0
        [pop2, fit2] = Migrate(pop1, fit1, pop2, fit2);
        [pop3, fit3] = Migrate(pop1, fit1, pop3, fit3);
    end

    if mod(gen, 10) == 0
        [pop1, fit1] = Migrate(pop2, fit2, pop1, fit1);
        [pop3, fit3] = Migrate(pop2, fit2, pop3, fit3);
    end

    if (mod(gen, 20) == 0) && (~exists)
        pop1 = Reset(popSize, numberOfGenes);
    end

    if (mod(gen, 10) == 0)
        pop2 = warming(pop2, 0.01, paramInterval);
    end

    parfor o = 1:popSize
        fit1(o) = Fitness(pop1(o,:));
        fit2(o) = Fitness(pop2(o,:));
        fit3(o) = Fitness(pop3(o,:));
    end %CYKLUS KAZDEHO JEDINCA
    
    %% GA Island 1
    [l, i] = min(fit1);
    evo1(gen) = l;
    oldPop1 = pop1;

    pop1 = GeneticFcn(pop1, fit1, paramInterval);

    %% GA Island 2
    [m1, j] = min(fit2);
    evo2(gen) = m1;
    oldPop2 = pop2;

    pop2 = GeneticFcn(pop2, fit2, paramInterval);

    %% GA Island 3
    [n, k] = min(fit3);
    fitValue = min(fit3)
    mojafit = fitValue;
    evo3(gen) = n;
    oldPop3 = pop3;

    pop3 = GeneticFcn(pop3, fit3, paramInterval);
    
    clf;
    hold on;
    plot(evo1, 'Color', 'red', 'LineWidth', 1);
    plot(evo2, 'Color', 'green', 'LineWidth', 1);
    plot(evo3, 'Color', 'blue', 'LineWidth', 1);
    hold off;
    xlim([0 maxGen]);
    drawnow();

    if gen == 10
        W1_10 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
        B1_10 = oldPop3(k,51:1:60);
        W2_10 = [oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90); oldPop3(k,91:1:100); oldPop3(k,101:1:110);
            oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150); oldPop3(k,151:1:160)];
        B2_10 = oldPop3(k,161:1:170);
        W3_10 = [oldPop3(k,171:1:180)' oldPop3(k,181:1:190)'];
    end

    if gen == 30 
        W1_30 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
        B1_30 = oldPop3(k,51:1:60);
        W2_30 = [oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90); oldPop3(k,91:1:100); oldPop3(k,101:1:110);
            oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150); oldPop3(k,151:1:160)];
        B2_30 = oldPop3(k,161:1:170);
        W3_30 = [oldPop3(k,171:1:180)' oldPop3(k,181:1:190)'];
    end

end

%% 2 vrstvy
% bestW1 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
% bestB1 = oldPop3(k,51:1:60);
% bestW2 = [oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90); oldPop3(k,91:1:100); oldPop3(k,101:1:110);
%     oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150); oldPop3(k,151:1:160)];
% bestB2 = oldPop3(k,161:1:170);
% bestW3 = [oldPop3(k,171:1:180)' oldPop3(k,181:1:190)'];

%% 2 vrstvy 9 lucov
bestW1 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)
            oldPop3(k,51:1:60); oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90)];
bestB1 = oldPop3(k,91:1:100);
bestW2 = [oldPop3(k,101:1:110); oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150);
    oldPop3(k,151:1:160); oldPop3(k,161:1:170); oldPop3(k,171:1:180); oldPop3(k,181:1:190); oldPop3(k,191:1:200)];
bestB2 = oldPop3(k,201:1:210);
bestW3 = [oldPop3(k,211:1:220)' oldPop3(k,221:1:230)'];

%% jedna vrstva 5 lucov
% bestW1 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
% bestB1 = oldPop3(k,51:1:60);
% bestW2 = [oldPop3(k,61:1:62); oldPop3(k,63:1:64); oldPop3(k,65:1:66); oldPop3(k,67:1:68); oldPop3(k,69:1:70);
% oldPop3(k,71:1:72); oldPop3(k,73:1:74); oldPop3(k,75:1:76); oldPop3(k,77:1:78); oldPop3(k,79:1:80)];


bestPop = pop3;
bestFit = fit3;
uloz.bestW1 = bestW1;
uloz.bestB1 = bestB1;
uloz.bestW2 = bestW2;
uloz.bestB2 = bestB2;
uloz.bestW3 = bestW3;
uloz.bestPop = bestPop;
uloz.bestFit = bestFit;
uloz.mojafit = mojafit;

filename = 'LatestSpecimen.mat';
save(filename, '-struct', 'uloz');

clf;
hold on;
plot(evo1, 'Color', 'red', 'LineWidth', 1);
plot(evo2, 'Color', 'green', 'LineWidth', 1);
plot(evo3, 'Color', 'blue', 'LineWidth', 1);
hold off;

evo3(end)
hold on;