clear all;
close all;

% Change the path to the rosbag file
bag = rosbag('/media/tmn/mySataSSD/DATASETS/NTU_VIRAL/eee_01/eee_01.bag');

% Image data
imgData = select(bag, 'Topic', '/left/image_raw');

% Pointcloud data
pcdData = select(bag, 'Topic', '/os1_cloud_node1/points');

% Read data
imgMsgs = readMessages(imgData);
pcdMsgs = readMessages(pcdData);

% Synchronize data
ts1 = timeseries(imgData);
ts2 = timeseries(pcdData);
t1 = ts1.Time;
t2 = ts2.Time;

k = 1;
if size(t2,1) > size(t1,1)
    for i = 1:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end   
end

if ~exist('img', 'dir')
    mkdir('img');
end
if ~exist('pcd', 'dir')
    mkdir('pcd');
end

% Save the data into files
for i = 1:length(idx)
    I = readImage(imgMsgs{idx(i,1)});
    pc = pointCloud(readXYZ(pcdMsgs{idx(i,2)}));
    n_strPadded = sprintf( '%04d', i ) ;
    pcdFileName = strcat('pcd','/', n_strPadded, '.pcd');
    imgFilename = strcat('img','/', n_strPadded, '.png');
    imwrite(I, imgFilename);
    pcwrite(pc, pcdFileName);
end