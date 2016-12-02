close all;

path = 'frame_0083.png';
crop_width = 500;
setpoint = 50;

response = imread(path);
[h, w] = size(response);
cropped_response = response(:,round(w/2.0)-crop_width/2.0:round(w/2.0)+crop_width/2.0);

mesh(cropped_response); title('Controller Output');
zlim([0 255]);

figure; error = abs(im2double(response) - setpoint/255.0);
mesh(error); title('Normalized Error (fraction of setpoint)');
zlim([0 1]);

figure; plot(mean(error), 'LineWidth', 2); title('Horizontal Error Profile');
ylim([0, 1]); grid on;
xlim([0, crop_width]);

figure; plot(mean(error, 2), 'LineWidth', 2); title('Vertical Error Profile');
ylim([0, 1]); grid on;
xlim([0, h]);