FROM ros:foxy

RUN apt-get update && \
    apt-get install -y \
        ros-foxy-mavros \
        ros-foxy-image-view \
        ros-foxy-cv-bridge \
        ros-foxy-joy \
        ros-foxy-camera-calibration \
        ros-foxy-rqt-console \
        libasio-dev \
        python3-rosinstall \
        python3-pip \
        python3-yaml \
        python3-rospkg \
        python3-catkin-pkg && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3 300
SHELL ["/bin/bash", "-c"]
WORKDIR /ros
COPY ./install_scripts ./install_scripts
RUN ./install_scripts/build_tello_ros.sh
COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt && mkdir src
COPY ./tello_ros ./src/tello_ros
RUN colcon build
COPY ./launch/ ./scripts/ ./camera_settings/camera.yaml ./

ENTRYPOINT ["/bin/bash", "-c", "./start.sh"]
