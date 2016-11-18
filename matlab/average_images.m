led_d = intensity('~/Desktop/led_images/frame0000.jpg');
i = 0;
for d = 41:120
    i = i + 1;
    s = sprintf('%4.4d',d);
    led_d(:,:,i) = intensity(['~/Desktop/led_images/frame',s,'.jpg']);
end

led_d = uint8(round(mean(led_d, 3)));