# Writeup for Camera Based 2D Feature Tracking
## Data Buffer
### MP.1 Data Buffer Optimization
If the frames are exceeding the limit, I simply delete the old frame when the new frame arrives.

## Keypoints
### MP.2 Keypoint Detection
The HARRIS detector is implemented with the cv::cornerHarris function,
followed with nms step to remove the close detection.
Other detectors are implemented in detKeypointsModern function.
### MP.3 Keypoint Removal
Use cv::Rect::contains function to judge if the point is inside our ROI.

## Descriptors
### MP.4 Keypoint Descriptors
Use cv::DescriptorExtractor to extract the features. See details in descKeypoints function.

### MP.5 Descriptor Matching
Select the distance type based on the feature type. Use L2 distance for SIFT,
and Hamming distance for binary descriptors. See details in matchDescriptors function.

### MP.6 Descriptor Distance Ratio
Use distance ratio 0.8 for KNN matching.

## Performance
### MP.7~9 Performance Evaluation 1,2,3
I run the detector and descriptor group in the Udacity workspace. The following table shows the result. <br/>

| Detector  | Descriptor | Keypoints | Avg detection time (ms) | Avg extraction time (ms) | Matches | Match per ms |
| ---       | ---        |       --- |                     --- |                      --- |     --- |          --- |
| SHITOMASI | BRISK      |       117 |                      19 |                      338 |      76 |     0.212885 |
| SHITOMASI | BRIEF      |       117 |                      14 |                        1 |      90 |            6 |
| SHITOMASI | ORB        |       117 |                      14 |                        0 |      85 |      6.07143 |
| SHITOMASI | FREAK      |       117 |                      12 |                       39 |      63 |      1.23529 |
| SHITOMASI | SIFT       |       117 |                      11 |                       15 |     103 |      3.96154 |
| HARRIS    | BRISK      |        24 |                      17 |                      338 |      11 |    0.0309859 |
| HARRIS    | BRIEF      |        24 |                      15 |                        0 |      15 |            1 |
| HARRIS    | ORB        |        24 |                      15 |                        0 |      15 |            1 |
| HARRIS    | FREAK      |        24 |                      16 |                       38 |      10 |     0.185185 |
| HARRIS    | SIFT       |        24 |                      16 |                       13 |      18 |      0.62069 |
| FAST      | BRISK      |       409 |                       1 |                      342 |     203 |     0.591837 |
| FAST      | BRIEF      |       409 |                       1 |                        1 |     242 |          121 |
| FAST      | ORB        |       409 |                       1 |                        1 |     229 |        114.5 |
| FAST      | FREAK      |       409 |                       1 |                       41 |     174 |      4.14286 |
| FAST      | SIFT       |       409 |                       1 |                       30 |     309 |      9.96774 |
| BRISK     | BRISK      |       276 |                     379 |                      340 |     144 |     0.200278 |
| BRISK     | BRIEF      |       276 |                     379 |                        1 |     149 |     0.392105 |
| BRISK     | ORB        |       276 |                     379 |                        4 |     103 |      0.26893 |
| BRISK     | FREAK      |       276 |                     380 |                       41 |     121 |     0.287411 |
| BRISK     | SIFT       |       276 |                     379 |                       41 |     182 |     0.433333 |
| ORB       | BRISK      |       116 |                       7 |                      338 |      72 |     0.208696 |
| ORB       | BRIEF      |       116 |                       7 |                        0 |      50 |      7.14286 |
| ORB       | ORB        |       116 |                       7 |                        4 |      58 |      5.27273 |
| ORB       | FREAK      |       116 |                       7 |                       39 |      38 |     0.826087 |
| ORB       | SIFT       |       116 |                       7 |                       47 |      84 |      1.55556 |
| SIFT      | BRISK      |       138 |                     130 |                      321 |      59 |      0.13082 |
| SIFT      | BRIEF      |       138 |                     132 |                        0 |      66 |          0.5 |
| SIFT      | FREAK      |       138 |                     130 |                       40 |      56 |     0.329412 |
| SIFT      | SIFT       |       138 |                     105 |                       80 |      88 |     0.475676 |
| AKAZE     | BRISK      |       167 |                      74 |                      340 |     123 |     0.297101 |
| AKAZE     | BRIEF      |       167 |                      72 |                        0 |     120 |      1.66667 |
| AKAZE     | ORB        |       167 |                      71 |                        2 |     102 |      1.39726 |
| AKAZE     | FREAK      |       167 |                      69 |                       41 |     108 |     0.981818 |
| AKAZE     | AKAZE      |       167 |                      73 |                       65 |     130 |     0.942029 |
| AKAZE     | SIFT       |       167 |                      69 |                       25 |     141 |          1.5 |

## Conclusion
As we can see in the table above. The top 3 detector-descriptor group is: <br/>
1. FAST+BRIEF 121 points per ms <br/>
2. FAST+ORB 114.5 points per ms <br/>
3. FAST+SIFT 9.9 points per ms <br/>

