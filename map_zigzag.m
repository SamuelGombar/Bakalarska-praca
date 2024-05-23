scenario = drivingScenario;
roadCenters = [0 0 0;
    7.5 -0.7 0;
    16.7 -35.3 0;
    31.5 1.4 0;
    43.4 -32.7 0;
    57.3 -3.6 0;
    64.8 -28.8 0;
    79.5 -6.5 0;
    86.6 -26.2 0;
    100.1 -10.5 0;
    115 -20 0];
roadWidth = 10;
headings = [0;NaN;NaN;NaN;NaN;NaN;NaN;NaN;NaN;NaN;0];
road(scenario, roadCenters, roadWidth, 'Heading', headings, 'Name', 'Road');

rb = roadBoundaries(scenario);
outerBoundary = rb{1};
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;
outerBoundary(153,1) = 250;
outerBoundary(154,1) = 250;

XFINISH1 = 115;
XFINISH2 = 119;
YFINISH1 = -15;
YFINISH2 = -25;

XLIMITS = [-5 120];
YLIMITS = [-50 20];