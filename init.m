%% Environment & road creation
scenario = drivingScenario;
roadCenters = [0 0; 80 0; 81 0; 81 -40; 80 -40; 70 -37; 60 -43; 50 -37; 40 -43; 30 -37; 21 -40; 20 -40; 20 -80; 21 -80; 100 -80];
roadWidth = 10;
road(scenario, roadCenters, roadWidth);

%% Actor
v1 = vehicle(scenario, 'ClassID',1', 'Position',[0 0 0], 'Velocity',[0.1 -0.5 0], 'Yaw',0);
v1.RearOverhang = 2.35;

%% Boundaries
rb = roadBoundaries(scenario);
outerBoundary = rb{1};
outerBoundary(181,1) = 150;
outerBoundary(182,1) = 150;
outerBoundary(2,1) = -50;
outerBoundary(3,1) = -50;

%% Radars
sensorLength = 15;
sensorFov = 5*pi/6;

%% Simulation
%REAL LENGTH IS 4.7, WIDTH IS 1.8

L = 4.7;
