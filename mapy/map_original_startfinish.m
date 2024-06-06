%start
plot([-4 -4 0 0 -4], [5 -5 -5 5 5], 'Color', 'black');
fill([-4 -4 0 0 -4], [5 -5 -5 5 5], 'green');

%finish
for j = 0:2:2
    flop = true;
    if j == 2 
        flop = false;
    end
    xfinish = [100+j 100+j 102+j 102+j 100+j];
    for i = 0:2.5:7.5
        yfinish = [-75-i -77.5-i -77.5-i -75-i -75-i];
        hplot = plot(xfinish, yfinish);
        hplot.Color = 'black';
        if flop == true
            fill(xfinish, yfinish, 'white');
            flop = false;
        else
            fill(xfinish, yfinish, 'black');
            flop = true;
        end
    end
end