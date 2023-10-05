% MATLAB controller for Webots
% File:          turn_right_around.m
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
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
count = 1;
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
left_sensor = wb_robot_get_device('left wheel sensor');
right_sensor = wb_robot_get_device('right wheel sensor');
gyro = wb_robot_get_device('gyro');
camera = wb_robot_get_device('camera');
distance_sensor = wb_robot_get_device('ps4');

wb_camera_enable(camera, TIME_STEP);
wb_gyro_enable(gyro, TIME_STEP);
wb_position_sensor_enable(left_sensor, TIME_STEP);
wb_position_sensor_enable(right_sensor, TIME_STEP);
wb_distance_sensor_enable(distance_sensor, TIME_STEP);

wb_motor_set_velocity(left_motor, 0.5*MAX_VELOCITY);
wb_motor_set_velocity(right_motor, 0.5*MAX_VELOCITY);
wb_motor_set_position(right_motor, 10);
wb_motor_set_position(left_motor, -10);

while wb_robot_step(TIME_STEP) ~= -1
  x_y_z_array = wb_gyro_get_values(gyro);
  angle = wb_position_sensor_get_value(left_sensor);
  distance = wb_distance_sensor_get_value(distance_sensor);
  
  %wb_console_print(sprintf('angular velocity %f\n', x_y_z_array(3)), WB_STDOUT);
  %wb_console_print(sprintf('angle of wheel %f\n', angle), WB_STDOUT);
  wb_console_print(sprintf('distance from obstacle %f\n', distance), WB_STDOUT);
  
  image = wb_camera_get_image(camera);
  str1 = 'C:\RoboticVision\Simulation\picture\_';
  str2 = num2str(count);
  str3 = '.png';
  name = strcat(str1,str2, str3);
  success = wb_camera_save_image(camera, name, 100)
  count = count + 1;
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
