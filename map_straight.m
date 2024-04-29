scenario = drivingScenario;
roadCenters = [0 0 0; 200 0 0];
roadWidth = 10;
road(scenario, roadCenters, roadWidth);

rb = roadBoundaries(scenario);
outerBoundary = rb{1};
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;
outerBoundary(103,1) = 250;
outerBoundary(104,1) = 250;

XFINISH1 = 200;
XFINISH2 = 204;
YFINISH1 = 5;
YFINISH2 = -5;

XLIMITS = [-10 210];
YLIMITS = [-100 100];