function pop = GeneticFcn(pop, fit, paramInterval, ~)
    best = selbest(pop, fit, 1);
    bestPop = selbest(pop, fit, [1 1 1]); %5
    rand = selrand(pop, fit, 15); %14
    sus = selsus(pop, fit, 31);
    pop = [rand; bestPop; sus];
    pop = mutx(pop, 0.1, paramInterval);
    %pop = muta(pop, 0.1, amp, paramInterval);
    pop = crossov(pop, 2, 0);
    pop = [best;pop];
end