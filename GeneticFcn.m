function pop = GeneticFcn(pop, fit, paramInterval, ~)
    amp = 0.01*ones(1, length(pop));
    best = selbest(pop, fit, 1);
    bestPop = selbest(pop, fit, [1 1 1]); %5
    rand = selrand(pop, fit, 15); %14
    sus = selsus(pop, fit, 31);
%     bestPop = crossov(bestPop, 2, 0);
    pop = [rand; bestPop; sus];
    pop = mutx(pop, 0.06, paramInterval);
    pop = muta(pop, 0.15, amp, paramInterval);
    pop = crossov(pop, 2, 0); %skus crossovat work groupy
    pop = [best;pop];
end