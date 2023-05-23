# Writeup: 3D Object Detection (Mid-Term Project)

The goal of the mid-term project was to make use of lidar measurements provided by a Velodine laser-scanner mounted on top of a vehicle, to detect objects in the surroundings of the vehicle's environment. This task was split into the following sub-tasks:

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

<p align="center"><img src="writeup/S1_F100_Range_Image.png"/></p>
<p align="center">Range (top) and intensity (bottom) channel [Sequence 1, Frame 100].</p>

<p align="center"><img src="writeup/S1_F100_Point_Cloud.png"/></p>
<p align="center">3D point cloud [Sequence 1, Frame 100].</p>

## Computing the bird's eye view from the 3D point cloud

## Object detection in the bird's eye view

<p align="center"><img src="writeup/S1_F100_Intensity_Map.png"/></p>
<p align="center">Intensity layer of the bird's eye view [Sequence 1, Frame 100].</p>

<p align="center"><img src="writeup/S1_F100_Height_Map.png"/></p>
<p align="center">Height layer of the bird's eye view [Sequence 1, Frame 100].</p>

<p align="center"><img src="writeup/S1_F100_Detected_Objects.png"/></p>
<p align="center">Detected objects [Sequence 1, Frame 100].</p>

## Performance evaluation of the detection algorithm

## Summary




Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

