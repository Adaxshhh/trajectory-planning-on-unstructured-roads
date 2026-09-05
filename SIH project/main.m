% main_setup.m

% 1. Define Ego Vehicle Parameters (Standard Indian Hatchback/Sedan)
ego.Length = 4.0; 
ego.Width = 1.7;
ego.FrontOverhang = 0.8;
ego.RearOverhang = 0.7;

% 2. Call your module to build the sensor suite
% The 'perception' prefix targets your package folder
mySensors = perception.getSensorSuite(ego.Length, ego.Width, ego.FrontOverhang, ego.RearOverhang);

% 3. Initialize Tracker
myTracker = multiObjectTracker('FilterInitializationFcn', @initcvekf, ...
    'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);

% 4. Open the Simulink Model
open_system('models/main_closed_loop.slx');