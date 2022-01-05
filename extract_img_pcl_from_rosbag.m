clear all;
close all;

% Change the path to the rosbag file
bag = rosbag('/media/tmn/mySataSSD/DATASETS/NTU_VIRAL/sbs_03/sbs_03.bag');

% Image data
imgData = select(bag, 'Topic', '/left/image_raw');

% Pointcloud data
pcdData = select(bag, 'Topic', '/os1_cloud_node1/points');

% Read data
imgMsgs = readMessages(imgData);
pcdMsgs = readMessages(pcdData);

% Synchronize data

% Extract the timestamp of the left cam images
tt_im = tic;
fprintf('Extracting image timestamps...\n');
t_im = zeros(size(imgMsgs, 1), 1);
for n=1:size(imgMsgs, 1)
    t_im(n) = imgMsgs{n}.Header.Stamp.seconds;
end
fprintf('Done: %f\n', toc(tt_im));

% Extract the timestamps of the poincloud
tt_pc = tic;
fprintf('Extracting poincloud timestamps...\n');
t_pc = zeros(size(pcdMsgs, 1), 1);
for n=1:size(pcdMsgs, 1)
    t_pc(n) = pcdMsgs{n}.Header.Stamp.seconds;
end
fprintf('Done: %f\n', toc(tt_pc));

% Match the time of the samples
[idx(:, 1), idx(:, 2)] = matchtime(t_im, t_pc, 0.05, 0.1);

% Create the directory
if ~exist('img', 'dir')
    mkdir('img');
end
if ~exist('pcd', 'dir')
    mkdir('pcd');
end

% Save the data into files
for i = 1:length(idx)
    I = readImage(imgMsgs{idx(i, 1)});
    pc = pointCloud(readXYZ(pcdMsgs{idx(i, 2)}));
    n_strPadded = sprintf( '%04d', i ) ;
    pcdFileName = strcat('pcd','/', n_strPadded, '.pcd');
    imgFilename = strcat('img','/', n_strPadded, '.png');
    imwrite(I, imgFilename);
    pcwrite(pc, pcdFileName);
end