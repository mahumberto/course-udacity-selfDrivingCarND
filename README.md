# Self-Driving Car Engineer Nanodegree
The Self-Driving Car Engineer Nanodegree is a 9 months program from Udacity in parnership with industry references on Autonomous Driving technologies, such as NVIDIA, Mercedes-Benz, OTTO and DiDi.

The program is divided on 3 terms, with the following main topics:
- Term 1: Computer Vision (ex: OpenCV, Image transformation, Gradients) and Deep Learning (ex: NNs, CNNs, Behavioral Cloning, TensorFlow, Keras) - [in-depht syllabus](https://medium.com/self-driving-cars/term-1-in-depth-on-udacitys-self-driving-car-curriculum-ffcf46af0c08)
- Term 2: Sensor Fusion (ex: Kallman Filters), Localization (ex: Particle Filters), Control - [in-depth syllabus](https://medium.com/udacity/term-2-in-depth-on-udacitys-self-driving-car-curriculum-775130aae502)
- Term 3: Under devellopment 

I am part of one the first cohorts from this Nanodegree program and am currently enroling Term 2. This repository contain my lesson notes, code and developed projects. 

## 1. Completed Projects 
### Term 1: Vehicle Detection and Tracking 
- Developed a pipeline to detect and track vehicles in a video stream from a front-facing camera on a car
- Built a Support Vector Machine (SVM) classifier for vehicle identification based on image color and gradients
- The pipeline successfully detects and track all vehicles distant up to 20m from the car
- [Project link](https://github.com/hmaleman/course-udacity-selfDrivingCarND/tree/master/term1-computerVision-DeepLearning/13-proj-vehicleDetection)

### Term 1: Advanced Lane Finding
- Developed a pipeline to detect lane boundaries in a video stream from a front-facing camera on a car 
- Used Computer Vision techniques to transform and filter images, identify lanes and calculate lane curvature 
- On daylight conditions, the pipeline detects lane lines independent of line color or pavement color change
- [Project link](https://github.com/hmaleman/course-udacity-selfDrivingCarND/tree/master/term1-computerVision-DeepLearning/11-proj-advancedLaneFinding)

### Term 1: Behavioral Cloning
- Developed a Deep Learning model with Keras for steering angle control of an autonomous car in a simulator
- Trained and integrated a NVIDIA’s CNN for behavioral cloning to predict steering angle based on 3 cameras
- With only 2 laps as input training, the model drives perfectly the same lap and generalizes well for other laps
- [Project link](https://github.com/hmaleman/course-udacity-selfDrivingCarND/tree/master/term1-computerVision-DeepLearning/09-proj-behavioralCloning)

### Term 1: Traffic Sign Classifier
- Developed a pipeline with Deep Learning with TensorFlow to identify German traffic signs
- Implemented and integrated a variation from the LeNet architecture as Deep Learning model
- The pipeline presented an accuracy of 94% without image augmentation. It served as great base to introduce TensorFlow
- [Project link](https://github.com/hmaleman/course-udacity-selfDrivingCarND/tree/master/term1-computerVision-DeepLearning/06-proj-trafficSignClassifier)

### Term 1: Basic Lane Finding
- Developed a basic pipeline to idenfiy lane lines in a video stream from front-facing camera on a car
- Implemented only basic Computer Vision techniques such as Canny Edge detection with the help of OpenCV
- As a Computer Vision introduction, this pipeline detects well lane lines, but it is limited and extrapolate only straight lane lines. 
- [Project link](https://github.com/hmaleman/course-udacity-selfDrivingCarND/tree/master/term1-computerVision-DeepLearning/02-proj-findingLaneLines)
