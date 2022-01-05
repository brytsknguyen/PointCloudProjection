# PointCloudProjection
Matlab script to demo the pointcloud projection on image for the [NTU VIRAL dataset](https://ntu-aris.github.io/ntu_viral_dataset/)

# How to use

1. Install latest Matlab. The script is written on Matlab R2021b. Earlier version may not have neccessary functions for interfacing with rosbag or pointcloud

2. Declare the path to the NTU VIRAL rosbag in extract_img_pcl_from_rosbag.m and run it.

3. After finishing the extraction, declare the image-pointcloud pair for projection in project_pcd_to_img.m. You can see the pointcloud projected on the image at the end.

Example:

![proj_0900](https://user-images.githubusercontent.com/13504903/145541180-184f9a47-2cd4-477e-b119-9b6a9b8bc5ba.jpg)
