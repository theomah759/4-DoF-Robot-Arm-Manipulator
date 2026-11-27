% --- Fixed calibration directory and file ---
calibDir = 'C:\Users\SReye\AppData\RBE3001_A25\RBE3001_A25_18\camera_calibration';
calibFile = fullfile(calibDir, 'cameraParams.mat');

% Ensure directory exists
if ~exist(calibDir, 'dir')
    fprintf('Creating calibration directory: %s\n', calibDir);
    mkdir(calibDir);
end

% --- If no valid calibration file exists, run camcalib first ---
if ~isfile(calibFile)
    fprintf('No cameraParams.mat found. Running camcalib to create one...\n');

    % Add calibration folder to MATLAB path (if not already)
    addpath(calibDir);

    % Run your calibration script (must define cameraParams)
    camcalib;

    % Verify cameraParams exists
    if exist('cameraParams', 'var')
        fprintf('Saving calibration parameters to:\n%s\n', calibFile);
        save(calibFile, 'cameraParams');
        fprintf('✅ Calibration saved successfully.\n');
    else
        error('❌ camcalib did not produce a variable named "cameraParams". Check camcalib.m.');
    end
else
    fprintf('Existing calibration file found at:\n%s\n', calibFile);
end

% --- Now safely create camera and image processor objects ---
cam = Camera(); % This will now find cameraParams.mat
imgProcessor = ImageProcessor(debug=true);
imgProcessor.cam = cam;
preview = webcam("USB Camera");

% --- Continue with image processing ---
mask = imgProcessor.generate_static_mask(margin=20);
imshow(mask);
title('Static Checkerboard Mask');

[u, v] = ginput(4); % Get 4 pixel coordinates
p_robot = imgProcessor.image_to_robot([u, v]); % Transform pixel -> robot base
