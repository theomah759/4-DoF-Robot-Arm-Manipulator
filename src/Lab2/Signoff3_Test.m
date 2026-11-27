robot = Robot();
% thetas and alphas are set to 0
dh_tab = [   0,   36.076,  0,   0;
             0,    60.25,  0,   -90;
          -79.38,    0, 130.231,0;
           79.38,    0,   124,  0;
             0,      0,  133.4, 0];
ht = robot.dh2fk(dh_tab);

disp(ht);