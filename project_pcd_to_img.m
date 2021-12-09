clear all;
close all;

% Procedure follows opencv tutorial
% https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html

% Distortion params
k1 = -0.288105327549552;
k2 =  0.074578284234601;
p1 =  7.784489598138802e-04;
p2 = -2.277853975035461e-04;

% Image sizes
img_width  = 752;
img_height = 480; 

% Intrinc params
fx = 4.250258563372763e+02;
fy = 4.267976260903337e+02;
cx = 3.860151866550880e+02;
cy = 2.419130336743440e+02;

% Intrinsic matrix
Mintr = [fx 0 cx; 0 fy cy];

% Extrinsic matricez
tf_B_Lhorz = [ 1.0,  0.0,  0.0, -0.05;...
               0.0,  1.0,  0.0,  0.00;...
               0.0,  0.0,  1.0,  0.055;...
               0.0,  0.0,  0.0,  1.0 ];
tf_B_Cam = [-0.01916508, -0.01496218,  0.99970437,  0.00519443;...
             0.99974371,  0.01176483,  0.01934191,  0.13478020;...
            -0.01205075,  0.99981884,  0.01473287,  0.01465067;...
             0.00000000,  0.00000000,  0.00000000,  1.00000000];

% Find the transform from lidar to camera
tf_Cam_B = tfinv(tf_B_Cam);
tf_Cam_Lhorz = tfmult(tf_Cam_B, tf_B_Lhorz);

% Read the image and pointcloud
img = imread('img/0900.png');
pcd = pcread('pcd/0900.pcd');

% Extract the xyz
xyz_inLhorz = pcd.Location';
xyz_inCam = tf_Cam_Lhorz(1:3, 1:3)*xyz_inLhorz + tf_Cam_Lhorz(1:3, 4);

xy_norm = [xyz_inCam(1, :)./xyz_inCam(3, :);...
           xyz_inCam(2, :)./xyz_inCam(3, :)];

% Apply the distortion, comment if you think not neccessary
[xy_norm(1, :), xy_norm(2, :)] = distort(xy_norm(1, :),...
                                         xy_norm(2, :),...
                                         k1, k2, p1, p2);

% Convert to pixel coordinates
uv = Mintr*[xy_norm(1, :);...
            xy_norm(2, :);...
            ones(1, length(xyz_inCam))];

% Find the coordinates of the pointcloud in the FOV
idx_FOV = find(0 < uv(1, :) ...
              & uv(1, :) < img_width - 1 ...
              & 0 < uv(2, :) ...
              & uv(2, :) < img_height - 1);

% Remove the points outside of the FOV
uv = round(uv(:, idx_FOV)) + 1;

% Mark the pixels with the pointcloud intensity
rgb_img = cat(3, img, img, img);
for n=1:length(uv)
    rgb_img(uv(2, n), uv(1, n), 1) = 255;
    rgb_img(uv(2, n), uv(1, n), 2) = 0;
    rgb_img(uv(2, n), uv(1, n), 3) = 0;
end

% Display the image with pointcloud overlay
imshow(rgb_img)