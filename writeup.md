# Writeup: 3D Object Detection (Mid-Term Project)

The goal of the mid-term project was to make use of lidar measurements provided by a Velodyne laser-scanner mounted on top of a vehicle, to detect objects in the surroundings of the vehicle's environment. This task was split into the following sub-tasks:

1) Computing the 3D point-cloud out of the range image provided by the laser-scanner
    - Extracting the intensity and height channel from the range image
    - Visualizing the 3D point-cloud
2) Computing the bird's eye view from the 3D point cloud
    - Converting 3D points into the bird's eye view
    - Computing the intensity layer
    - Computing the height layer
3) Object detection in the bird's eye view
    - Evaluating FPN ResNet for object detection
    - Extracting the bounding boxes from the model output
4) Performance evaluation of the detection algorithm
    - Computing the intersection-over-union between labels and detections
    - Computing the number of false-negatives and false-positives
    - Computing precision and recall

Below, a brief description of the different steps as well as some results can be found.

## Computing the 3D point-cloud out of the range image provided by the laser-scanner

The starting point for detecting objects in measurements created by a laser-scanner is the so called range image. It is basically a panoramic view of the scene, which encodes the intensity of the reflected light, the distance, and the elongation of a 3D point.

In this task, the range and the intensity channel of the range image had to be extracted. One very important step is to identify points without a valid reflection, those are marked with the value "-1" and have to be removed from further processing steps. Another very important step is to reduce the impact of highly reflecting objects. Those can create very high intensity values which would make the other reflections "invisible" in the intensity map as their value is too low. In the implementation, the percentiles P1 and P99 have been applied. Hence, intensity values with a value lower than P1 have been set to the P1 value. Intensity values higher than P99 have been set to the P99 value. Finally, the values had to be normalized.

Another information which can be extracted from the range image is the distance of the 3D points w.r.t. the laser-scanner. As for the intensity layer, invalid values had to be identified and removed and the remaining values had to be normalized.

Examples of the resulting intensity and range layers can be seen in Figure 1.

<p align="center"><img src="writeup/S1_F100_Range_Image.png"/></p>
<p align="center">Figure 1: Range (top) and intensity (bottom) channel [Sequence 1, Frame 100].</p>

The next stop of the processing chain was to visualize the 3D point cloud based. The points themselves where available already, the visualization had to be implemented. This was done using the Open3D toolbox.

An example of a 3D point cloud can be seen in Figure 2.

<p align="center"><img src="writeup/S1_F100_Point_Cloud.png"/></p>
<p align="center">Figure 2: 3D point cloud [Sequence 1, Frame 100].</p>

## Computing the bird's eye view from the 3D point cloud

Many object detection algorithms which rely on machine learning methods take as input the bird's eye view of the environment. This is basically a grid with cells representing a certain area of the vehicle's environment. Each cell contains the following information:
- Height of the highest 3D point falling into this cell
- Intensity of the brightest 3D point falling into this cell
- Amount of 3D points falling into the cell

As the bird's eye view is a grid with three different values for each cell, it can be treated as an image with three channels (i.e. and RGB image).

The goal of this task was to create the intensity and height layer for each cell. For this purpose, the points of the 3D point cloud falling into each cell had to be identified and the highest point as well as the brightest point in each cell had to be determined. This information had to be stored in the respective layer of the bird's eye view map.

An example of the intensity layer of the bird's eye view can be seen in Figure 3, Figure 4 shows the corresponding height layer.

<p align="center"><img src="writeup/S1_F100_Intensity_Map.png"/></p>
<p align="center">Figure 3: Intensity layer of the bird's eye view [Sequence 1, Frame 100].</p>

<p align="center"><img src="writeup/S1_F100_Height_Map.png"/></p>
<p align="center">Figure 4: Height layer of the bird's eye view [Sequence 1, Frame 100].</p>

## Object detection in the bird's eye view

Combining the layers mentioned above with the layer containing the information about the amount of 3D points falling into each cell of the grid results in the final bird's eye view as shown in Figure 5. Please be aware that the bird's eye view in Figure 5 is rotated by 180 degrees compared to the individual layers shown in Figures 3 and 4.

<p align="center"><img src="writeup/S1_F100_BEV.png"/></p>
<p align="center">Figure 5: Bird's eye view [Sequence 1, Frame 100].</p>

The bird's eye view shown in Figure 5 is taken as input for the machine learning method used to detect objects in the surroundings of the vehicle. In this task, a different CNN had to be used in order to learn how to integrate different models into the detection pipeline. In particular, the "FPN ResNet" had to be integrated into the processing pipeline.

Most challenging in that task was to find out which configuration parameters are needed by the approach and how to interpret the output of the model.

Figure 6 (top) shows the camera image with an overlay visualizing the labeled objects. The color of the 3D bounding boxes encodes whether a label is valid or not. Green bounding boxes represent valid labels, blue bounding boxes represent invalid labels. However, the reason why those labels are considered invalid is unclear. As the bird's eye view covers a longitudinal distance up to 50 m and only the three closest vehicles are visible in the bird's eye view, the assumption is that the other vehicles have a distance larger than 50 m and are not considered anymore in the Waymo Open Dataset which is limited to a detection range of 50 m.

The bottom part of Figure 6 shows the bird's eye view of the scene with the detected objects as overlay. As can be seen from the image, the detection match the present objects pretty good in this particular frame.

<p align="center"><img src="writeup/S1_F100_Detected_Objects.png"/></p>
<p align="center">Figure 6: Detected objects [Sequence 1, Frame 100].</p>

## Performance evaluation of the detection algorithm

To be able to evaluate the performance of an object detection algorithm, the criteria used is usually the intersection-over-union. In this task, the calculation of the intersection-over-union had to be implemented. Based on this information, it is possible to determine whether a detection is a true positive or not. This is the case if the intersection-over-union exceeds a certain threshold.

Additionally, two more values had to be calculated. On the one hand, the number of false-negatives, i.e. labeled objects which have not been detected by the object detection algorithm. On the other hand the false-positives, i.e. detections of the algorithm were not real object is present.

Using those numbers, two important performance metrics for object detection algorithms can be calculated: Precision and Recall

These two metrics can be used to evaluate the performance of an object detection algorithm and to compare different algorithms against each other.

## Summary
