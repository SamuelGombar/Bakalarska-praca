roadCenters = [0 0; 1 0; 100 -30; 40 -80; 50 -30; 69 -40; 70 -40];
roadWidth = 10;
road(scenario, roadCenters, roadWidth);

rb = roadBoundaries(scenario);
outerBoundary = rb{1};
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;
outerBoundary(161,1) = 90;
outerBoundary(162,1) = 90;

%% FINISH
% for j = 0:2:2
%     flop = true;
%     if j == 2 
%         flop = false;
%     end
%     xfinish = [70+j 70+j 72+j 72+j 70+j];
%     for i = 0:2.5:7.5
%         yfinish = [-35-i -37.5-i -37.5-i -35-i -35-i];
%         hplot = plot(xfinish, yfinish);
%         hplot.Color = 'black';
%         if flop == true
%             fill(xfinish, yfinish, 'white');
%             flop = false;
%         else
%             fill(xfinish, yfinish, 'black');
%             flop = true;
%         end
%     end
% end