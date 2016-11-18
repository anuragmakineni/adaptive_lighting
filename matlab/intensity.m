function [ img ] = intensity( image_path )
    img = rgb2gray(imread(image_path));
end

