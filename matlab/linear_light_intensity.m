%% Set up script
clc, clear, close all;

generate = false;
crop_width = 200;
path = '~/Desktop/linear_fit_images/frame_';

%% Calculate light averages
if generate
    
    % Level number = %on/10
    level1=average_intensity(0, 34, 71, path);
    level2=average_intensity(0, 72, 108, path);
    level3=average_intensity(0, 109, 140, path);
    level4=average_intensity(0, 141, 195, path);
    level5=average_intensity(0, 197, 224, path);
    level6=average_intensity(0, 225, 250, path);
    level7=average_intensity(0, 251, 282, path);
    level8=average_intensity(0, 283, 311, path);
    level9=average_intensity(0, 312, 349, path);
    level10=average_intensity(0, 350, 398, path);
    
    % Crop Images
    level1 = level1(:,1024-crop_width:1024+crop_width);
level2 = level2(:,1024-crop_width:1024+crop_width);
level3 = level3(:,1024-crop_width:1024+crop_width);
level4 = level4(:,1024-crop_width:1024+crop_width);
level5 = level5(:,1024-crop_width:1024+crop_width);
level6 = level6(:,1024-crop_width:1024+crop_width);
level7 = level7(:,1024-crop_width:1024+crop_width);
level8 = level8(:,1024-crop_width:1024+crop_width);
level9 = level9(:,1024-crop_width:1024+crop_width);
level10 = level10(:,1024-crop_width:1024+crop_width);
    
else
    load('linear_image_averages.mat');
end

%% Calculate linearity

% Find averages
avg1 = mean(mean(level1));
avg2 = mean(mean(level2));
avg3 = mean(mean(level3));
avg4 = mean(mean(level4));
avg5 = mean(mean(level5));
avg6 = mean(mean(level6));
avg7 = mean(mean(level7));
avg8 = mean(mean(level8));
avg9 = mean(mean(level9));
avg10 = mean(mean(level10));

avgs = [avg1 avg2 avg3 avg4 avg5 avg6 avg7 avg8 avg9 avg10]';
input_value = (.1:.1:1)';

% Find the fit
fit = fit(input_value, avgs, 'poly1');

% Plot the data
figure; hold on;

plot(input_value, avgs, 'ko');
plot(fit);
title('Average Intensity Value of Images at Different Light Levels')
xlabel('Input Light Value')
ylabel('Intensity')