function popTo = Migrate(popFrom, fitFrom, popTo, fitTo)
    migrateBest = selbest(popFrom, fitFrom, 1);
    [~, index] = max(fitTo);
    popTo(index, :) = migrateBest;
end