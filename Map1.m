roadCenters = [0 0; 80 0; 81 0; 81 -40; 80 -40; 70 -37; 60 -43; 50 -37; 40 -43; 30 -37; 21 -40; 20 -40; 20 -80; 21 -80; 100 -80];
roadWidth = 10;
road(scenario, roadCenters, roadWidth);

rb = roadBoundaries(scenario);
outerBoundary = rb{1};
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;
outerBoundary(181,1) = 150;
outerBoundary(182,1) = 150;

%% FINISH
% for j = 0:2:2
%     flop = true;
%     if j == 2 
%         flop = false;
%     end
%     xfinish = [100+j 100+j 102+j 102+j 100+j];
%     for i = 0:2.5:7.5
%         yfinish = [-75-i -77.5-i -77.5-i -75-i -75-i];
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