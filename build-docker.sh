#!/bin/bash

# Path to the directory containing the Dockerfile
DOCKERFILE_PATH="."

# Name and tag for the Docker image
IMAGE_NAME="pallet:ros-humble-jetson"

# Build the Docker image
docker build -t ${IMAGE_NAME} ${DOCKERFILE_PATH}

echo "Image ${IMAGE_NAME} built successfully."