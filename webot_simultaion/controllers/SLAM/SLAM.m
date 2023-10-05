% MATLAB controller for Webots
% File:          SLAM.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

MAX_SPEED = 6.28;

% initialize devices
ps = [];
ps_names = [ "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" ];

for i = 1:8
  ps(i) = wb_robot_get_device(convertStringsToChars(ps_names(i)));
  wb_distance_sensor_enable(ps(i), TIME_STEP);
end
left_sensor = wb_robot_get_device('left wheel sensor');
right_sensor = wb_robot_get_device('right wheel sensor');
wb_position_sensor_enable(left_sensor, TIME_STEP);
wb_position_sensor_enable(right_sensor, TIME_STEP);
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0.0);
wb_motor_set_velocity(right_motor, 0.0);

% get the display (not a real e-puck device !)
%display_device = wb_robot_get_device('display');
%wb_display_set_color(display_device, [0 0 0]);

% get and enable camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera,TIME_STEP);
%camera.setFocalDistance(0.01) #设置焦距

while wb_robot_step(TIME_STEP) ~= -1
  
  
  
  
  
  
  %避障
  % read sensors outputs
  ps_values = [];
  for i = 1:8
    ps_values(i) = wb_distance_sensor_get_value(ps(i));
  end

  % detect obstacles
  right_obstacle = ps_values(1) > 80.0 | ps_values(2) > 80.0 | ps_values(3) > 80.0;
  left_obstacle = ps_values(6) > 80.0 | ps_values(7) > 80.0 | ps_values(8) > 80.0;

  % initialize motor speeds at 50% of MAX_SPEED.
  left_speed  = 0.8 * MAX_SPEED;
  right_speed = 0.8 * MAX_SPEED;
  % modify speeds according to obstacles
  if left_obstacle
    % turn right
    left_speed   = 0.8 * MAX_SPEED;
    right_speed  = -0.8 * MAX_SPEED;
  elseif right_obstacle
    % turn left
    left_speed  = -0.8 * MAX_SPEED;
    right_speed = 0.8 * MAX_SPEED;
  end
  % write actuators inputs
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
  
  
  
  
  

  %影像處理
  rgb = wb_camera_get_image(camera);

  green = rgb(:,:,3);
  %[image_height,image_width] = size(green);
  image_width = wb_camera_get_width(camera)
  image_height = wb_camera_get_height(camera)
 
  green = 255-green


  for j = 1:image_width
    if max(find(green(:,j) >240)) >0
      dis1(j) = max(find(green(:,j) >240));
      dis(j) = 11.494*exp(-0.017*dis1(j))
      theta(j) = 0.1509*j-24.215;
    else
      dis(j) = 0;
      theta(j) = 0.1509*j-24.215;
    end
  end
  
  %影像處回傳
  img_output = [dis, theta];
  %輪軸編碼器回傳
  wheel_angle = [wb_position_sensor_get_value(left_sensor),  wb_position_sensor_get_value(right_sensor)];
  %wb_console_print(sprintf('left wheel %f\n', wheel_angle(1)), WB_STDOUT);


  

  %wb_console_print(sprintf('right wheel %f\n', dis(160)), WB_STDOUT);
  
  
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
