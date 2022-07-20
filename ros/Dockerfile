FROM ros:foxy

RUN apt-get update && \
    apt-get install -y \
        ros-foxy-mavros \
        ros-foxy-image-view \
        ros-foxy-cv-bridge \
        libasio-dev \
        python3-rosinstall \
        python3-pip \
        python3-yaml \
        python3-rospkg \
        python3-catkin-pkg

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 300
SHELL ["/bin/bash", "-c"]
COPY ./scripts ./scripts
RUN ./scripts/build_tello_ros.sh
WORKDIR /ros
RUN source /opt/ros/foxy/setup.bash && mkdir -p ./src && cd src && ros2 pkg create --build-type ament_python tello_ros
# COPY ./src ./src
RUN colcon build
COPY . .

ENTRYPOINT ["/bin/bash", "-c", "./start.sh"]
