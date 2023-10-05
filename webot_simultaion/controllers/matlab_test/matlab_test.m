% MATLAB controller for Webots
% File:          matlab_test.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;
MAX_VELOCITY = 6.28;

% get and enable devices, e.g.:
camera = wb_robot_get_device('camera');
distance_sensor = wb_robot_get_device('ps0');
wb_distance_sensor_enable(distance_sensor, TIME_STEP);
wb_camera_enable(camera, TIME_STEP);
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
wb_motor_set_position(right_motor, inf);
wb_motor_set_position(left_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);
while wb_robot_step(TIME_STEP) ~= -1
  distance = wb_distance_sensor_get_value(distance_sensor);
  wb_console_print(sprintf('distance from obstacle %f\n', distance), WB_STDOUT);
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
wb_motor_set_velocity(left_motor, 0.5*MAX_VELOCITY);
wb_motor_set_velocity(right_motor, 0.5*MAX_VELOCITY);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
