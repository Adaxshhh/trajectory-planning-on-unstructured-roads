% +perception/getSensorSuite.m
function sensors = getSensorSuite(egoLength, egoWidth, frontOverhang, rearOverhang)
    
    % 1. Calculate dynamic offsets based on function inputs
    wb = egoLength - frontOverhang - rearOverhang;
    halfWidth = egoWidth / 2;
    
    % 2. Initialize the array
    sensors = cell(8,1);
    
    % Front facing long range radar
    sensors{1} = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174, ...
        'SensorLocation', [wb + frontOverhang, 0], 'FieldOfView', [20,5]);
        
    % Rear facing long range radar
    sensors{2} = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180, ...
        'SensorLocation', [rearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20,5]);
        
    % Rear left facing short range radar
    sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120, ...
        'SensorLocation', [0, halfWidth], 'MaxRange', 30, 'ReferenceRange', 50, ...
        'FieldOfView', [90,5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
        
    % Rear right facing short range radar
    sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120, ...
        'SensorLocation', [0, -halfWidth], 'MaxRange', 30, 'ReferenceRange', 50, ...
        'FieldOfView', [90,5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
        
    % Front left facing short range radar
    sensors{5} = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60, ...
        'SensorLocation', [wb, halfWidth], 'MaxRange', 30, ...
        'ReferenceRange', 50, 'FieldOfView', [90,5], 'AzimuthResolution', 10, ...
        'RangeResolution', 1.25);
        
    % Front right facing short range radar
    sensors{6} = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60, ...
        'SensorLocation', [wb, -halfWidth], 'MaxRange', 30, ...
        'ReferenceRange', 50, 'FieldOfView', [90,5], 'AzimuthResolution', 10, ...
        'RangeResolution', 1.25);
        
    % Front facing camera
    sensors{7} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
        'SensorLocation', [0.75 * wb, 0], 'Height', 1.1);
        
    % Rear facing camera
    sensors{8} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
        'SensorLocation', [0.2 * wb, 0], 'Height', 1.1, 'Yaw', 180);
end