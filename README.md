# Investigation into surface roughness characterization using LiDAR and cameras

## Introduction
This repository stores all the files relating to my final year project for my BSc Eng Electrical and Computer Engineering degree at the University of Cape Town which was completed in 2021 and supervised by Robyn Verrinder and James Hepworth.

## Abstract
This thesis investigates the use of LiDAR and camera technology for the purpose of characterization of surface roughness. The application of this research is to eventually be used to characterize sea ice in the Marginal Ice Zone near Antarctica. This would assist in understanding the dynamics of the sea ice due to the effect surface roughness has on the drift of sea ice. This research provides a preliminary investigation by analyzing current literature in surface roughness characterization of sea ice and optimal point cloud processing algorithms. 
A point cloud dataset is gathered from the Velodyne VLP-16 on the Clearpath Husky UGV through rigorously designed experiments using 3D printed surfaces of known roughness. The data collected provides a set of varied position parameters in the X, Y and Z directions through the manipulation of external experiment parameters. 
A point cloud processing pipeline is suggested and the preprocessing of this methodology is demonstrated in MATLAB. The plane extraction uses the robust RANSAC Algorithm the positional relationship between physical and point cloud measurements is done using the Iterative Closest Point Algorithm (ICP). 
This methodology yielded a good model from the point cloud data with the best results at distance less than 1m.

## Report
The full report can be found in pdf format in this repository.

## LiDAR dataset:
Since the LiDAR dataset collected was extremely large, it is stored in this Google drive folder:
https://drive.google.com/drive/folders/1Y39fgW_kSC2XKlZ9mpXWKRPv9tUatTl-?usp=sharing

## Stl files
The 3D print designs are found in the stl folder in a .stl format

## Code
The processing code is found in all_process.mlx. The other code is supplementary preprocessing. 


