# peer_segmentation_deployment

 Project for detecting and segmenting Pallets and ground in a warehouse. Contains steps for training and deploying the model on Nvidia edge device using Docker

#### System and tools Used: AMD Ryzen 7 with 8GB RAM, Ubuntu 20.04, Docker

### Dataset 
Received an un-annotated dataset of 519 images, these are Monocular-rectified images of a warehouse taken by a zed-2i camera, Images are of shape 416X416. Link: https://drive.google.com/drive/folders/1xSqKa55QrNGufLRQZAbp0KFGYr9ecqgT 

### Data Annotation with Label-Studio and RoboFlow
The given dataset is unannotated for Pallets and ground, so I started to use Label-studio for Annotation. Label-Studio has the option to take assistance from LVM's such as Grounding-SAM, DINOv2 in their backend.

#### follow these steps to set SAM in Label-Studio for data annotation

reference:https://github.com/HumanSignal/label-studio-ml-backend

Step1: install label studio in a local virtual environment and start the web interface
```
pip install label-studio
label-studio start
```
step2: clone Label-studio ML backend and run the ML model in docker container
```
git clone https://github.com/HumanSignal/label-studio-ml-backend.git
cd label-studio-ml-backend/label_studio_ml/examples/segment_anything_model/
```
step3: connect the ML model with the web interface using API-KEY, make the changes in docker-compose.yml file

step4: follow this video to link the model, data and start the annotation with SAM assistance: https://github.com/HumanSignal/label-studio-ml-backend/tree/master/label_studio_ml/examples/segment_anything_model

I encountered an issue with compute on a CPU, my models kept giving blank results in the backend, Finally I used Roboflow for annotation, roboflow has SAM integrated and it runs on their cloud backend

Roboflow projects are public for free tier **Roboflow Project ID: pallets_segmentation-xof0g**

the datasets are still a work in progress

the final RoboFlow Dataset can be obtained with the following Python code

```
from roboflow import Roboflow
rf = Roboflow(api_key="nCKuh1QV2SflFsBOlwz2")
project = rf.workspace("sasankprojects").project("pallets_segmentation-xof0g")
version = project.version(3)
dataset = version.download("yolov11")
```
### Training

I used YOLOv11-nano to perform object detection and Segmentation, this can be found in the Jupyter Notebook in train folder


### Deploy

#### File Structure
```
This Repository/
├── Dockerfile        # Creates a Dockerfile that runs on Nvidia
├── Dockerfile_X86.txt        # Dockerfile I used to run ROS2 and onnx
├── run-docker.sh        # bash commands to start the docker container with Nvidia GPU
├── build-docker.sh        # bash commands to build the docker image
├── lab_studio_docker.sh        # code to start label-studio web interface as a dockercontainer

├── train/        # directory with jupyter notebook to train the model
    ├── runs/        # contains the iterations performed for segmentation and Object detection
    ├── train_yolo.ipynb        # contains the code to train the models, and convert to other deployment formats

├── peer_ws/src/        # directory contains packages for segmentation and object detection
    ├── pallet_detection_python/        # directory to deploy the object detection in Python with GPU enabled
        ├── pallet_detection.py        # Python code for the publisher node
    ├──pallet_detection_RT/        # package to deploy the object detection model in C++ with ONNX, no GPU
        ├── pallet_onnx.cpp        # C++ code for the object detection publisher node
```

#### Instructions to run the Publisher node on Nvidia AGX

clone this repo on the edge device and build the Docker image with ROS2, Ultralytics, onnxruntime and TensorRT 
```
git clone https://github.com/sasank98/peer_segmentation_deployment.git
cd peer_segmentation_deployment
bash build-docker.sh
```
Start the container, this automatically opens the ROS2_workspace and build the packages in the docker container
```
bash run-docker.sh
colcon build
```
Run the following command to start the node that subscibes to the RGB image of left_camera in Zed2i
```
ros2 run pallet_detection_python pallet_detection
```

#### NOTE

this Repo is still a work in progress, more segmentation models and object detection models will be added. This repo also need to have code to deploy the segmentation model

