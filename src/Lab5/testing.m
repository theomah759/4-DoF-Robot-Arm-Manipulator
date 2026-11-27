% robot = Robot();
trajGen = TrajGenerator();
imgProcessor = ImageProcessor();
camera = Camera;

%robot.openGripper();
% robot.blocking_ts_move([281.4009 0 224.3263 0],"time",5);
%robot.blocking_ts_move([112 0 60 90],"time",5);
coords = [112, 90];

% rawimg = imgProcessor.generate_static_mask();
% disp("place balls")
% pause;
% XYimg = imgpro.image_to_robot(rawimg);
% XYrobot = imgpro.detect_centroids(XYimg, "min_size", 20);
% XYZimg = imgpro.correct_centroids(XYrobot, "ball_z", 25);
% %coords = detect_balls();


% robot.pick_up_ball(coords, "red", "z_offset", 100);

disp camera.getCameraInstrinsics
