function [ avg ] = average_intensity( init, start, end_index, path )
    d = init;
    s = sprintf('%4.4d',d);
    img = imread([path,s,'.png']);
    
    i = 0;
    for d = start:end_index
        i = i + 1;
        s = sprintf('%4.4d',d);
        img(:,:,i) = imread([path,s,'.png']);
    end

    avg = uint8(round(mean(img, 3)));
end