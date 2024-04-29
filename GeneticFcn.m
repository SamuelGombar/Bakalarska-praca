function pop = GeneticFcn(pop, fit, paramInterval)
    amp = 0.01*ones(1, length(pop));
    amp1 = 0.001*ones(1, length(pop));
    best = selbest(pop, fit, 1);
    bestPop = selbest(pop, fit, [1 1 1]); %5
    rand = selrand(pop, fit, 15); %14
    sus = selsus(pop, fit, 16);
    work1 = seltourn(pop, fit, 15);
    work2 = crossov([sus; work1], 2, 0);
%     bestPop = crossov(bestPop, 2, 0);
    pop = [rand; bestPop; work2];
    pop = mutx(pop, 0.06, paramInterval);
    pop = muta(pop, 0.15, amp, paramInterval);
    pop = muta(pop, 0.15, amp1, paramInterval);
    pop = [best;pop];
end