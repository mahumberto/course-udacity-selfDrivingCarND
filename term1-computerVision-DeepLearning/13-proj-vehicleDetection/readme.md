# Project: Vehicle Detection and Tracking

## Goal
The goal of this project was to develop a software pipeline to process a video stream from a front-mounted car camera, detect and track vehicles in it.  

## Applied methodology
### Classifier
Initially a classifier was trained to identify 64x64px vehicle / non-vehicle images. The used input image database was ___. For speed and simplicty resons, a Support Vector Machine model was used for the classifier. Several image features were extracted for the model input, in special, color channel histogram and Hogh gradient for shape identification disregarding the color.

### Pipeline
From the video stream, each frame provided an input which was processed to extract the same image features used for training. The areas of interest in the frame (side lanes and horizon of the road) were then framented on 3 sized sliding windows with different strides. Each window was fed to the SVM model in order to form a heat map of vehicle probability. The map was then filtered in order to remove false positives from the heat map and provided the next frame processing a specific region of interest.

## Results
20-output-projectVideo.mp4
- YouTube link

## Keywords:
Machine Learning, Computer Vision, Hogh gradient, Support Vector Machine
