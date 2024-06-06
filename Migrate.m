function [popTo, fitTo] = Migrate(popFrom, fitFrom, popTo, fitTo)
    migrateBest = selbest(popFrom, fitFrom, 1);
    [~, index] = max(fitTo);
    popTo(index, :) = migrateBest;
    fitTo(index) = min(fitFrom);
end