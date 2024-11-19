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

I used YOLOv11-nano to perform object detection and Segmentation, this can be found in the train folder




