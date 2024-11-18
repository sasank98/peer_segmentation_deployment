#!/bin/bash
# Always run from cobra-docker directory (./run-docker.sh)

# Name of the Docker image
IMAGE_NAME="pallet:ros-humble"

# Name of the Docker container
CONTAINER_NAME="ros2_humble"

HOST_HOME="$(pwd)/peer_ws/"
CTNR_HOME="/home/peer_ws/"

HOST_SRC="$(realpath .)"
CTNR_SRC="$CTNR_HOME"

# Check if the container is already running
if [ $(docker ps -q -f name=${CONTAINER_NAME}) ]; then
  echo "Container ${CONTAINER_NAME} is already running."
else
  # Run the container
    docker run --runtime nvidia -it --rm \
        --name ${CONTAINER_NAME} \
        --privileged \
        --net=host \
        --volume="${HOST_HOME}:${CTNR_HOME}" \
        --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw" \
        ${IMAGE_NAME} /bin/bash
fi
# --env="DISPLAY" \
# --env="QT_X11_NO_MITSHM=1" \
# --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
# --volume="/etc/localtime:/etc/localtime:ro" \
        # --volume="${HOST_SRC}:${CTNR_SRC}" \
