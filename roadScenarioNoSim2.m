%% Clearing
close all;
clear;
clc;

maxGen = 100;
popSize = 20;
numberOfGenes = 190;
paramInterval = [ones(1,numberOfGenes)*-3; ones(1,numberOfGenes)*3];
amp = 0.01*ones(1, numberOfGenes);
fit = zeros(1, popSize);
pop = zeros(popSize, numberOfGenes);
% pop = ones(popSize, numberOfGenes);
% pop = genrpop(popSize, paramInterval);
evo = zeros(1, maxGen);

for gen = 1:maxGen

    parfor o = 1:popSize
        fit(o) = fitness(pop(o,:));
        disp(gen);
    end %CYKLUS KAZDEHO JEDINCA
    
    %% GENETIC ALGORITHM
    [v, i] = min(fit);
    min(fit)
    evo(gen) = v;
    oldPop = pop;

    best = selbest(pop, fit, 1);
    best1 = selbest(pop, fit, [1 1 1 1]); %5
    rand = selrand(pop, fit, 15); %14
    pop = [rand; best1];
    pop = mutx(pop, 0.1, paramInterval);
    pop = muta(pop, 0.1, amp, paramInterval);
    pop = crossov(pop, 2, 0);
    pop = [best;pop];

    if gen == 15
        W1_15 = [oldPop(i,1:1:10); oldPop(i,11:1:20); oldPop(i,21:1:30); oldPop(i,31:1:40); oldPop(i, 41:1:50)];
        W2_15 = [oldPop(i,51); oldPop(i,52); oldPop(i,53); oldPop(i,54); oldPop(i,55); oldPop(i,56); oldPop(i,57); oldPop(i,58); oldPop(i,59); oldPop(i,60)];
        B1_15 = [oldPop(i, 61:1:numberOfGenes)];
    end

    if gen == 50 
        W1_50 = [oldPop(i,1:1:10); oldPop(i,11:1:20); oldPop(i,21:1:30); oldPop(i,31:1:40); oldPop(i, 41:1:50)];
        B1_50 = oldPop(i,51:1:60);
        W2_50 = [oldPop(i,61:1:70); oldPop(i,71:1:80); oldPop(i,81:1:90); oldPop(i,91:1:100); oldPop(i,101:1:110);
            oldPop(i,111:1:120); oldPop(i,121:1:130); oldPop(i,131:1:140); oldPop(i,141:1:150); oldPop(i,151:1:160)];
        B2_50 = oldPop(i,161:1:170);
        W3_50 = [oldPop(i,171:1:180)' oldPop(i,181:1:190)'];
    end

end

bestW1 = [oldPop(i,1:1:10); oldPop(i,11:1:20); oldPop(i,21:1:30); oldPop(i,31:1:40); oldPop(i, 41:1:50)];
bestB1 = oldPop(i,51:1:60);
bestW2 = [oldPop(i,61:1:70); oldPop(i,71:1:80); oldPop(i,81:1:90); oldPop(i,91:1:100); oldPop(i,101:1:110);
    oldPop(i,111:1:120); oldPop(i,121:1:130); oldPop(i,131:1:140); oldPop(i,141:1:150); oldPop(i,151:1:160)];
bestB2 = oldPop(i,161:1:170);
bestW3 = [oldPop(i,171:1:180)' oldPop(i,181:1:190)'];

plot(evo, 'Color', 'red');
evo(end)
hold on;