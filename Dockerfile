FROM ultralytics/ultralytics:latest-jetson-jetpack5


ARG ROS_DISTRO=humble
ARG ROS_PACKAGE=ros_base
ARG ROS_ROOT=/opt/ros/$ROS_DISTRO

ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_PACKAGE=${ROS_PACKAGE}
ENV ROS_ROOT=${ROS_ROOT}

# Below code is obtained from official Jetson-containers repository
# https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/ros2_build.sh

# Step 1: Install basic dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates

# Step 2: Add ROS2 repository and install development packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libbullet-dev \
        libpython3-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-numpy \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-rosinstall-generator \
        libasio-dev \
        libtinyxml2-dev \
        libcunit1-dev

# Step 3: Install pip packages
RUN pip3 install --upgrade --no-cache-dir \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest && \
    python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir scikit-build && \
    pip3 install --upgrade --no-cache-dir --verbose cmake && \
    cmake --version && \
    which cmake

# Step 4: Remove other versions of Python3
# ultralytics uses python3.8 so this line is optional
RUN apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal" && \
    ls -ll /usr/bin/python*

# Step 5: Create ROS_ROOT directory and download ROS sources
RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PACKAGE} \
        launch_xml \
        launch_yaml \
        launch_testing \
        launch_testing_ament_cmake \
        demo_nodes_cpp \
        demo_nodes_py \
        example_interfaces \
        camera_calibration_parsers \
        camera_info_manager \
        cv_bridge \
        v4l2_camera \
        vision_opencv \
        vision_msgs \
        image_geometry \
        image_pipeline \
        image_transport \
        compressed_image_transport \
        compressed_depth_image_transport \
        rosbag2_storage_mcap \
        rmw_fastrtps \
    > ros2.${ROS_DISTRO}.${ROS_PACKAGE}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PACKAGE}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PACKAGE}.rosinstall

# Step 6: Clone ament_cmake and handle patches for building Humble on 18.04
RUN rm -r ${ROS_ROOT}/src/ament_cmake && \
    git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO} && \
    SKIP_KEYS="libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    if [ "$ROS_DISTRO" = "humble" ] || [ "$ROS_DISTRO" = "iron" ] && [ $(lsb_release --codename --short) = "bionic" ]; then \
        SKIP_KEYS="$SKIP_KEYS rti-connext-dds-6.0.1 ignition-cmake2 ignition-math6" && \
        apt-get install -y --no-install-recommends gcc-8 g++-8 && \
        export CC="/usr/bin/gcc-8" && \
        export CXX="/usr/bin/g++-8" && \
        echo "CC=$CC CXX=$CXX" && \
        apt-get purge -y pybind11-dev && \
        pip3 install --upgrade --no-cache-dir pybind11-global && \
        git -C /tmp clone -b yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp.git && \
        cmake -S /tmp/yaml-cpp -B /tmp/yaml-cpp/BUILD -DBUILD_SHARED_LIBS=ON && \
        cmake --build /tmp/yaml-cpp/BUILD --parallel $(nproc --ignore=1) && \
        cmake --install /tmp/yaml-cpp/BUILD && \
        rm -rf /tmp/yaml-cpp; \
    fi && \
    echo "--skip-keys $SKIP_KEYS"

# Step 7: Install dependencies using rosdep
RUN rosdep init && \
    rosdep update && \
    rosdep install -y \
        --ignore-src \
        --from-paths src \
        --rosdistro ${ROS_DISTRO} \
        --skip-keys "$SKIP_KEYS"

# Step 8: Build ROS2
RUN colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Step 9: Cleanup
RUN rm -rf ${ROS_ROOT}/src && \
    rm -rf ${ROS_ROOT}/logs && \
    rm -rf ${ROS_ROOT}/build && \
    rm ${ROS_ROOT}/*.rosinstall && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Step 10: Source ROS2 setup.bash
RUN echo "source ${ROS_ROOT}/install/setup.bash" >> /root/.bashrc

WORKDIR /home/peer_ws