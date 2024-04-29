scenario = drivingScenario;
roadCenters = [0 0; 1 0; 152.2 -61.3; 132.5 -104; 19.5 -112.2; 149 -200 ;150 -200];
roadWidth = 10;
road(scenario, roadCenters, roadWidth);

rb = roadBoundaries(scenario);
outerBoundary = rb{1};
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;
outerBoundary(283,1) = 200;
outerBoundary(284,1) = 200;

XFINISH1 = 150;
XFINISH2 = 154;
YFINISH1 = -195;
YFINISH2 = -205;

XLIMITS = [-5 158];
YLIMITS = [-210 7];