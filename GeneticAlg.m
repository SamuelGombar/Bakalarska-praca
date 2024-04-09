%% Clearing
close all;
clear;
clc;

maxGen = 100;
popSize = 50;
numberOfGenes = 190;
paramInterval = [ones(1,numberOfGenes)*-3; ones(1,numberOfGenes)*3];
amp = 0.1*ones(1, numberOfGenes);

%% Island 1
fit1 = zeros(1, popSize);
pop1 = zeros(popSize, numberOfGenes);

%% Island 2
fit2 = zeros(1, popSize);
pop2 = zeros(popSize, numberOfGenes);

%% Island 3
fit3 = zeros(1, popSize);
pop3 = zeros(popSize, numberOfGenes);

for gen = 1:maxGen
    if mod(gen, 15) == 0
        pop1 = Migrate(pop2, fit2, pop1, fit1);
        pop3 = Migrate(pop2, fit2, pop3, fit3);
        pop2 = warming(pop2, 0.2, paramInterval);
    end

    if mod(gen, 30) == 0
        pop2 = Migrate(pop1, fit1, pop2, fit2);
        pop3 = Migrate(pop1, fit1, pop3, fit3);
        pop1 = Reset(popSize, numberOfGenes);
    end

    parfor o = 1:popSize
        fit1(o) = Fitness(pop1(o,:));
        fit2(o) = Fitness(pop2(o,:));
        fit3(o) = Fitness(pop3(o,:));
        disp(gen);
    end %CYKLUS KAZDEHO JEDINCA
    
    %% GA Island 1
    [l, i] = min(fit1);
    evo1(gen) = l;
    oldPop1 = pop1;

    pop1 = GeneticFcn(pop1, fit1, paramInterval, amp);

    %% GA Island 2
    [m, j] = min(fit2);
    evo2(gen) = m;
    oldPop2 = pop2;

    pop2 = GeneticFcn(pop2, fit2, paramInterval, amp);

    %% GA Island 3
    [n, k] = min(fit3);
    fitValue = min(fit3)
    evo3(gen) = n;
    oldPop3 = pop3;

    pop3 = GeneticFcn(pop3, fit3, paramInterval, amp);
    
    clf;
    hold on;
    plot(evo1, 'Color', 'red', 'LineWidth', 1);
    plot(evo2, 'Color', 'green', 'LineWidth', 1);
    plot(evo3, 'Color', 'blue', 'LineWidth', 1);
    hold off;
    xlim([0 100]);
    drawnow();

    if gen == 15
        W1_15 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
        B1_15 = oldPop3(k,51:1:60);
        W2_15 = [oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90); oldPop3(k,91:1:100); oldPop3(k,101:1:110);
            oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150); oldPop3(k,151:1:160)];
        B2_15 = oldPop3(k,161:1:170);
        W3_15 = [oldPop3(k,171:1:180)' oldPop3(k,181:1:190)'];
    end

    if gen == 50 
        W1_50 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
        B1_50 = oldPop3(k,51:1:60);
        W2_50 = [oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90); oldPop3(k,91:1:100); oldPop3(k,101:1:110);
            oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150); oldPop3(k,151:1:160)];
        B2_50 = oldPop3(k,161:1:170);
        W3_50 = [oldPop3(k,171:1:180)' oldPop3(k,181:1:190)'];
    end

end

bestW1 = [oldPop3(k,1:1:10); oldPop3(k,11:1:20); oldPop3(k,21:1:30); oldPop3(k,31:1:40); oldPop3(k, 41:1:50)];
bestB1 = oldPop3(k,51:1:60);
bestW2 = [oldPop3(k,61:1:70); oldPop3(k,71:1:80); oldPop3(k,81:1:90); oldPop3(k,91:1:100); oldPop3(k,101:1:110);
    oldPop3(k,111:1:120); oldPop3(k,121:1:130); oldPop3(k,131:1:140); oldPop3(k,141:1:150); oldPop3(k,151:1:160)];
bestB2 = oldPop3(k,161:1:170);
bestW3 = [oldPop3(k,171:1:180)' oldPop3(k,181:1:190)'];

clf;
hold on;
plot(evo1, 'Color', 'red', 'LineWidth', 1);
plot(evo2, 'Color', 'green', 'LineWidth', 1);
plot(evo3, 'Color', 'blue', 'LineWidth', 1);
hold off;

evo3(end)
hold on;