% control step
TIME_STEP=64;


% motor speed unit
SPEED_UNIT=0.00628;



% get the display (not a real e-puck device !)
display_device = wb_robot_get_device('display');
wb_display_set_color(display_device, [0 0 0]);

% get and enable camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera,TIME_STEP);
%camera.setFocalDistance(0.01) #设置焦距

% avoid dummy values on first device query
step = 0;
samples = 0;

wb_console_print('Hello!', WB_STDOUT);
wb_console_print(strcat('Running', ANSI_RED_FOREGROUND, ' Matlab', ANSI_RESET, ' sample Webots controller.'), WB_STDOUT);

f = fopen('result.txt','w')
while wb_robot_step(TIME_STEP) ~= -1
  a = num2str(step)
  fprintf(f,'\n %s \n',a)
  step = step + 1;


  % get camera RGB image
  % this function return an image in true color (RGB) uint8 format
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

  for i =1:image_width
    fprintf(f,'%7.3f ',theta(i));
  end
  fprintf(f,'\n');
  for i = 1:image_width
    fprintf(f,'%7.3f ',dis(i));
  end
  
  % display inversed rgb image with Display device
  inverse = 255 - rgb;
  imageref = wb_display_image_new(display_device, inverse, WB_IMAGE_RGB);
  wb_display_image_paste(display_device, imageref, 0, 0, false);
  wb_display_image_delete(display_device, imageref);

  % add gunsight lines
  wb_display_draw_line(display_device, 0, 19, 51, 19);
  wb_display_draw_line(display_device, 25, 0, 25, 38);

 
  % display camera image
  subplot(1,2,1);
  image(rgb);
  title('RGB Camera');

  % display 'canny' image
  subplot(1,2,2);
  image(green);
  %colormap('gray');
  title('green');


  % flush graphics
  drawnow;

 
end

% your cleanup code goes here
