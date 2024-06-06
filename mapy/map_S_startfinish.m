%start
plot([-4 -4 0 0 -4], [5 -5 -5 5 5], 'Color', 'black');
fill([-4 -4 0 0 -4], [5 -5 -5 5 5], 'green');

%finish
for j = 0:2:2
    flop = true;
    if j == 2 
        flop = false;
    end
    xfinish = [150+j 150+j 152+j 152+j 150+j];
    for i = 0:2.5:7.5
        yfinish = [-195-i -197.5-i -197.5-i -195-i -195-i];
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