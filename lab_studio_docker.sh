#!/bin/bash

docker run -it -p 8080:8080 \
               -v $(pwd)/mydata:/label-studio/data \
               --env LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true \
               --env LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT=/home/sasank/peer_take_home/Pallets \
               --user $(id -u):$(id -g) \
               heartexlabs/label-studio:latest
