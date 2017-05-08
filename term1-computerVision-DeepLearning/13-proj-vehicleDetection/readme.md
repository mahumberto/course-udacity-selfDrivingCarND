# Project: Vehicle Detection and Tracking

## 1. Goal
The goal of this project was to develop a software pipeline to process a video stream from a front-mounted car camera, detect and track vehicles in it.  

## 2. Applied methodology
### 2.1. Classifier
Initially a classifier was trained to identify 64x64px vehicle / non-vehicle images. The used input image database was ... . For speed and simplicty resons, a Support Vector Machine model was used for the classifier. Several image features were extracted for the model input, in special, color channel histogram and Hogh gradient for shape identification disregarding the color.

The classifier training code and documentation is present at the notebook: [01-proj-vehicleDetection.ipynb](https://github.com/hmaleman/course-udacity-selfDrivingCarND/blob/master/term1-computerVision-DeepLearning/13-proj-vehicleDetection/01-proj-vehicleDetection-classifier.ipynb).

### 2.2. Pipeline
From the video stream, each frame provided an input which was processed to extract the same image features used for training. The areas of interest in the frame (side lanes and horizon of the road) were then framented on 3 sized sliding windows with different strides. Each window was fed to the SVM model in order to form a heat map of vehicle probability. The map was then filtered in order to remove false positives from the heat map and provided the next frame processing a specific region of interest.

The described pipeline code, documentation and dicussions are summarized at the jupyter notebook: [20-proj-vehicleDetection-pipeline.ipynb](https://github.com/hmaleman/course-udacity-selfDrivingCarND/blob/master/term1-computerVision-DeepLearning/13-proj-vehicleDetection/02-proj-vehicleDetection-pipeline.ipynb).

## 3. Results
The current result was achieved after a 20h project run. It covers all basic requirements and still has a lot of room for improvement. However for the purpose of learning and implementing a sliding window associated with a ML model, this result is already fine.

The resul video is available in this repository: [20-output-projectVideo.mp4](https://github.com/hmaleman/course-udacity-selfDrivingCarND/blob/master/term1-computerVision-DeepLearning/13-proj-vehicleDetection/20-output-projectVideo.mp4)

And also at YouTube:

[![Vehicle Detection and Tracking](https://img.youtube.com/vi/y0NVArxNrOg/hqdefault.jpg)](https://www.youtube.com/watch?v=y0NVArxNrOg "Vehicle Detection and Tracking - Click to Watch!")

## 4. Keywords:
Machine Learning, Computer Vision, Hogh gradient, Support Vector Machine
