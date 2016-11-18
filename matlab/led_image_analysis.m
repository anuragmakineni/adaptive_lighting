close all;

generate = false;
crop_width = 200;
path = '~/Desktop/led_images/frame';

if generate
    led_d = average_intensity(0, 41, 120, path);
    both = average_intensity(0, 121, 217, path);
    led_c = average_intensity(0, 218, 305, path);
else
    load('image_averages.mat');
end

led_d = led_d(:,1024-crop_width:1024+crop_width);
both = both(:,1024-crop_width:1024+crop_width);
led_c = led_c(:,1024-crop_width:1024+crop_width);

[height, width] = size(led_c);

setpoint = 100 * ones(height, width);

imshow(led_d); title('LED D'); figure; imshow(led_c); title('LED C'); figure; imshow(both); title('BOTH'); figure;


sum = led_d + led_c;
mesh(both); zlim([0, 255]); title('Both Activated'); figure;
mesh(sum); zlim([0, 255]); title('Linear Sum'); figure;
mesh(sum - both); zlim([0, 10]); title('Error'); figure;
mesh(led_c); zlim([0, 255]); title('LED C'); figure;
mesh(led_d); zlim([0, 255]); title('LED D');
disp(['Average Error: ', num2str(mean2(sum - both))]);
disp(['Standard Deviation of Error: ', num2str(std2(sum - both))]);

%% least squares optimization
led_c_response = reshape(led_c, [], 1);
led_d_response = reshape(led_d, [], 1);
A = horzcat(led_c_response, led_d_response);

goal = reshape(setpoint, [], 1);

x = lsqr(double(A), double(goal));

figure; mesh(x(1)*led_c + x(2)*led_d); zlim([0, 255]); title('LSQR');
figure; mesh(uint8(round(x(1)*led_c + x(2)*led_d)) - uint8(setpoint)); zlim([0, 255]); title('LSQR Error');
figure; imshow((x(1)*led_c + x(2)*led_d)); title('Least Squares Solution');