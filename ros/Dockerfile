FROM ros:foxy

RUN apt-get update && \
    apt-get install -y \
        ros-foxy-mavros \
        ros-foxy-image-view \
        ros-foxy-cv-bridge \
        ros-foxy-joy \
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
RUN pip3 install -r requiremetns.txt
RUN mkdir src
COPY ./tello_ros ./src/tello_ros
RUN colcon build
COPY ./launch/ ./scripts/ ./

ENTRYPOINT ["/bin/bash", "-c", "./start.sh"]
