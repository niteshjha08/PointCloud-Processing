# PointCloud-Processing
Pipeline to process pointclouds from scratch

### Overview of steps:
#### Filtering:
- Range based filtering
- Angle based filtering
#### Downsampling
- Voxel grid based
- Fusing multiple pointclouds
    * Interesting problem to tackle: How to do this at high speeds (correct slewing and compensate ego-motion)?
#### Ground filtering
- RANSAC
- Ray ground filtering
#### Object detection
- Euclidean clustering using K-d tree
- DBSCAN
#### Shape Extraction
- Axis aligned BB
- EigenBox
- Oriented BB
- Convex Hull
#### Using these shapes as ROIs:
- Tracking
- Classification (on images)
- Collision checking
