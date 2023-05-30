# Writeup: 3D Object Detection (Mid-Term Project)

The goal of the mid-term project was to make use of lidar measurements provided by a Velodyne laser-scanner, mounted on top of a vehicle, to detect objects in the environment the vehicle is driving in. This task was split into the following sub-tasks:

1) Computing the 3D point-cloud out of the range image provided by the laser-scanner
    - Extracting the intensity and height channel from the range image
    - Visualizing the 3D point-cloud
2) Computing the bird's eye view from the 3D point cloud
    - Converting 3D points into the bird's eye view
    - Computing the intensity layer
    - Computing the height layer
3) Object detection in the bird's eye view
    - Evaluating "FPN ResNet" for object detection
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
<p align="center">Figure 1: Range (top) and intensity (bottom) layer [Sequence 1, Frame 100].</p>

The next step of the processing chain was to visualize the 3D point cloud. The points themselves were available already, the visualization had to be implemented. This was done using the Open3D toolbox.

An example of a 3D point cloud can be seen in Figure 2.

<p align="center"><img src="writeup/S1_F100_Point_Cloud.png"/></p>
<p align="center">Figure 2: 3D point cloud [Sequence 1, Frame 100].</p>

## Computing the bird's eye view from the 3D point cloud

Many object detection algorithms which rely on machine learning methods take as input the bird's eye view of the environment. This is basically a grid with cells representing a certain area of the vehicle's environment. Each cell contains the following information:
- Height of the highest 3D point falling into this cell
- Intensity of the brightest 3D point falling into this cell
- Total amount of 3D points falling into this cell

As the bird's eye view is a grid with three different values for each cell, it can be treated as an image with three channels (i.e. a RGB image).

The goal of this task was to create the intensity and height layer for each cell. For this purpose, the points of the 3D point cloud falling into each cell had to be identified and the highest point as well as the brightest point in each cell had to be determined. This information had to be stored in the respective layer of the bird's eye view. In order to get rid of extremely bright points, the P99 percentile (actually 99.5 %) has been used. This value has been used to limit the intensity values.

An example of the resulting intensity layer of the bird's eye view can be seen in Figure 3, Figure 4 shows the corresponding height layer.

<p align="center"><img src="writeup/S1_F100_Intensity_Map.png"/></p>
<p align="center">Figure 3: Intensity layer of the bird's eye view [Sequence 1, Frame 100].</p>

<p align="center"><img src="writeup/S1_F100_Height_Map.png"/></p>
<p align="center">Figure 4: Height layer of the bird's eye view [Sequence 1, Frame 100].</p>

## Object detection in the bird's eye view

Combining the layers mentioned above with the layer containing the information about the total amount of 3D points falling into each cell of the grid results in the final bird's eye view as shown in Figure 5. Please be aware that the bird's eye view in Figure 5 is rotated by 180 degrees compared to the individual layers shown in Figures 3 and 4.

<p align="center"><img src="writeup/S1_F100_BEV.png"/></p>
<p align="center">Figure 5: Bird's eye view [Sequence 1, Frame 100].</p>

The bird's eye view shown in Figure 5 is taken as input for the machine learning method used to detect objects in the surroundings of the vehicle. In this task, a different CNN had to be used in order to learn how to integrate different models into the detection pipeline. In particular, the "FPN ResNet" had to be integrated into the processing pipeline and the results had to be interpreted in the right way.

Figure 6 (top) shows the camera image with an overlay visualizing the labeled objects. The color of the 3D bounding boxes encodes whether a label is considered valid or not. Green bounding boxes represent valid labels, blue bounding boxes represent invalid labels. However, the reason why those labels are considered invalid is unclear. As the bird's eye view covers a longitudinal distance up to 50 m and only the three closest vehicles are visible in the bird's eye view, the assumption is that the other vehicles have a distance larger than 50 m and are not considered anymore in the Waymo Open Dataset, which is artificially limited to a detection range of 50 m.

The bottom part of Figure 6 shows the bird's eye view of the scene with the detected objects as overlay. As can be seen from the image, the detections match the present objects pretty good in this particular frame.

<p align="center"><img src="writeup/S1_F100_Detected_Objects.png"/></p>
<p align="center">Figure 6: Detected objects [Sequence 1, Frame 100].</p>

## Performance evaluation of the detection algorithm

To be able to evaluate whether a detection belongs to a labeled object or not, the criteria used is usually the intersection-over-union. In this task, the calculation of the intersection-over-union had to be implemented. Based on this information, it is possible to determine whether a detection is a true positive or not. This is the case if the intersection-over-union exceeds a certain threshold.

Additionally, two more values had to be calculated. On the one hand, the number of false-negatives, i.e. labeled objects which have not been detected by the object detection algorithm. On the other hand the false-positives, i.e. detections of the algorithm were no real object is present.

Using those numbers, two important performance metrics for object detection algorithms can be calculated: Precision and Recall.

These two metrics can be used to evaluate the performance of an object detection algorithm and to compare different algorithms against each other.

Figures 7-9 show the detection performance of the algorithm evaluated on the three sequences used in this project.

<p align="center"><img src="writeup/S1_Detection_Performance.png"/></p>
<p align="center">Figure 7: Detection performance for sequence 1.</p>

<p align="center"><img src="writeup/S2_Detection_Performance.png"/></p>
<p align="center">Figure 8: Detection performance for sequence 2.</p>

<p align="center"><img src="writeup/S3_Detection_Performance.png"/></p>
<p align="center">Figure 9: Detection performance for sequence 3.</p>

## Examples of vehicles with different degrees of visibility

<p align="center"><img src="writeup/S1_F0_Point_Cloud.png"/></p>
<p align="center">Figure 10: Vehicles with mainly the rear visible and a close vehicle with rear and side visible [Sequence 1, Frame 0].</p>

<p align="center"><img src="writeup/S1_F3_Point_Cloud.png"/></p>
<p align="center">Figure 11: Vehicles with mainly the front and side visible [Sequence 1, Frame 3].</p>

<p align="center"><img src="writeup/S1_F49_Point_Cloud_1.png"/></p>
<p align="center">Figure 12: Good visibility of the front and the side of the vehicle [Sequence 1, Frame 49].</p>

<p align="center"><img src="writeup/S2_F100_Point_Cloud.png"/></p>
<p align="center">Figure 13: Vehicle mainly seen from the side and one in far distance from the rear [Sequence 2, Frame 100].</p>

<p align="center"><img src="writeup/S3_F0_Point_Cloud_1.png"/></p>
<p align="center">Figure 14: Decreasing visibility depending on the distance of the vehicles [Sequence 3, Frame 0].</p>

<p align="center"><img src="writeup/S3_F30_Point_Cloud.png"/></p>
<p align="center">Figure 15: Decreasing visibility depending on the distance of the vehicles (vehicles passed the ego-vehicle) [Sequence 3, Frame 30].</p>

## Examples of stable vehicle features

Based on the 3D point clouds shown in Figures 10-15 as well as the intensity layer extracted from the range image (see Figure 1, bottom), the following features appear rather stable:
- Roof of the vehicles: They can be clearly identified as they are usually the highest part of the vehicles. Hence, they stand out in the 3D point cloud.
- Window of the vehicles: The intensity of the reflected light is pretty low due to the transparency of the windows. Hence, they appear dark in the intensity layer.
- Body of the vehicles: As the body of the vehicles is usually made up of metal, it also shows a high reflectivity in the intensity layer of the range image.

Prominent features in the 3D point cloud which remain stable across different frames are shown Figure 16 and Figure 17.

<p align="center"><img src="writeup/S1_F49_Point_Cloud_2_Detail_modified.png"/></p>
<p align="center">Figure 16: Prominent features in the 3D point cloud [Sequence 1, Frame 49].</p>

<p align="center"><img src="writeup/S3_F0_Point_Cloud_2_Detail_modified.png"/></p>
<p align="center">Figure 17: Prominent features in the 3D point cloud [Sequence 3, Frame 0].</p>

# Writeup: Sensor Fusion and Object Tracking (Final Project)

The goal of the final project was to implement a processing pipeline to detect and track multiple objects over time using a sensor fusion algorithm. This task was split into the following sub-tasks:

1) Implementation of an extended Kalman filter
2) Implementation of the track management
3) Implementation of the data association
4) Implementation of the measurement model for the camera

Below, a brief description of the different steps as well as some results can be found.

## Implementation of an extended Kalman filter

In the first sub-task an extended Kalman filter, i.e. the prediction and update step of the filter, had to be implemented. As the required equations where provided in the theoretical part of the training, it was basically straightforward to do the implementation.

The most challenging part of this task was the derivation of the covariance matrix of the process noise (Q), as this matrix had to be derived doing some simple math. Again, the theoretical foundation was provided during the training. Hence, this sub-task was also straightforward.

## Implementation of the track management

The second sub-task was related to the implementation of a track management, i.e. the initialization of new tracks and the handling of already existing tracks. To be able to decide whether a track shall be kept or not, two main attributes were considered for each track, the track score, i.e. the confidence of a track being created on a real object, as well as the track state.

Based on this information it was decided whether a track will be kept or deleted. Deletion usually happens when a track is not updated with new measurements anymore.

## Implementation of the data association

As long as only one track and only one measurement is available, it is fairly easy to do the data association. In case that multiple tracks and multiple sensor measurements are present, it has to be decided which measurement to associate with which track. This is usually done during a data association step which had to be implemented in the third sub-task.

The starting point of the data association was the derivation of the association matrix, i.e. a matrix which contains a metric defining the distance between a track and a measurement. In this project, the Mahalanobis distance has been used.

Based on the association matrix, a simple approach called "Simple Nearest Neighbor (SNN)" has been used to do the association of a measurement to a track. All tracks which have been updated with a measurement got an increased track score, i.e. the confidence of the track increased. In case a track did not get an update, the score was reduced. As soon as the score of a track dropped below a threshold, the track got deleted. Sensor measurement which have not been assigned to a track where used to create new tracks.

To avoid associating a measurement to a track which is too far away, a gating has been implemented.

## Implementation of the measurement model for the camera

So far, measurements from a laser-scanner have been used only. In the fourth sub-task, the processing pipeline had to be extended to camera measurements. Hence, creating a sensor fusion system.

The main activity in this step was the implementation of the non-linear measurement function, i.e. the projection of 3D world points onto the image. Additionally, a check whether a tracked object can be seen by the camera or not had to be implemented. This is required to decrease the score of a track, as this shall only be done of the object would be visible in general.

## Summary of the implementation

In general, the implementation of all sub-tasks was pretty straightforward. This was basically for two reasons. On the one hand, the framework of the processing pipeline was given already and the different tasks were described in the code. This made it extremely easy to implement the missing parts. On the other hand, basically all concepts have been introduced during the lessons and have partly been covered by the coding examples already.

## Benefits of sensor fusion

In general, fusion systems do have benefits compared to systems using a single sensor. In fusion systems, benefits of one sensor can be used to overcome shortcomings of other sensors. At nighttime for example, a camera does have a reduced performance as it relies on light reflected from objects. A laser-scanner for example does not suffer this issue as it is an active sensor. Hence, combining those sensors will enable more scenarios to be covered.

The same basically holds true for the visible field of view. The Velodyne laser-scanner used in this project is mounted on top of the vehicle. Hence, the close surroundings of the vehicle are not visible in the sensor's measurements. The camera can create measurements in the close surroundings of the vehicle and thus allows to detect objects in this area as well.

Having different sensors which all contribute to the track score will reduce the time a track gets confirmed which will enable an earlier usage of the tracks in other vehicle functions like adaptive cruise control or an emergence brake assist.

Regarding the detection accuracy, there is no significant improvement visible in the sequence used during this project. Figure 18 shows the RMSE of the system using the laser-scanner only, Figure 19 shows the RMSE of the fusion system. As can be seen, the RMSE of one track is slightly worse, the RMSE of the two other tracks is slightly better (please be aware that the track IDs differ in the two graphs).

In Figure 19, a major drawback of the used setup can be seen. Track 11 corresponds to a vehicle which does have a longitudinal distance of about 50 m to the ego-vehicle. Thus, it is at the visibility range of the laser-scanner (artificially limited to 50 m in the Waymo Open Dataset). The track is initialized using measurements coming from the laser-scanner. Afterwards it is only detected by the camera which cannot properly estimate the longitudinal distance of the vehicle, resulting in a rather high RMSE of that track.

<p align="center"><img src="writeup/S1_F0_to_200_RMSE.png"/></p>
<p align="center">Figure 18: RMSE of tracked object (lidar only) [Sequence 1, Frames 0-200].</p>

<p align="center"><img src="writeup/S1_F0_to_200_RMSE_Fusion.png"/></p>
<p align="center">Figure 19: RMSE of tracked object (lidar and camera) [Sequence 1, Frames 0-200].</p>

## Challenges of sensor fusion systems

As mentioned in the previous section already and shows in Figure 19, challenges arise in case a measurement from a sensor with a high position accuracy is used to initialize but then the object cannot be seen by that sensor anymore. If the object is seen by another sensor with a lower position accuracy, e.g. a camera, the estimated position is rather inaccurate and could trigger an emergency brake system erroneously for example. Figure 20 shows such a scenario where track 11 has been set up using measurements from the laser-scanner but then the track leaves the field of view and is kept due to camera measurements only resulting in a bad position estimate.

<p align="center"><img src="writeup/S1_F70_Object_Tracking.png"/></p>
<p align="center">Figure 20: Tracked objects (lidar and camera) [Sequence 1, Frame 70].</p>

Additionally, the computation burden of processing the data from many sensors might be pretty high. This results in a high power consumption.

## Improvements of the tracking results

There are a couple of improvements possible to increase the tracking performance of the fusion system:

- Parameter tuning: Tuning the parameters used might help to improve the tracking performance of the sensor fusion system.
- Use vehicle's ego-motion: So far, the ego-motion of the vehicle is not considered although it does have an impact on the predicted position of the tracked vehicles.
- Use more sophisticated motion model: The motion model used in the prediction step is rather generic. It does not consider constraints vehicles face while driving, e.g. that they cannot suddenly move to the side. This could for example be considered using a kinematic bicycle motion model.
- Use combination of different motion models: Usually, it is hard to find a good parameter set for different kinds of motion (e.g., constant velocity, acceleration,...). Making use of an interacting multiple model Kalman filter could be used to take different "motions" into consideration.
- Use more parameters: Currently, only the position and the velocity of the tracked objects is being used. It would be beneficial to add more parameters to the state vector. Especially the yaw angle might help to reduce sudden jumps in the object's orientation.
- Use more sophisticated Kalman filter: As the camera model used is a non-linear model, it could be beneficial to make use of an unscented Kalman filter which usually can handle non-linear systems better than the extended Kalman filter can.
- Use more sophisticated data association method: In the scenarios used in the project, the data association was not too challenging. Especially when having many vehicles present in the scene, the "Simple Nearest Neighbor (SNN)" approach might not lead to good results. A more sophisticated approach could be used to solve this issue.
