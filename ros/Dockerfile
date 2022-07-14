FROM ros:noetic-robot-buster

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && \
    apt-get install -y \
        ros-noetic-mavros \
        ros-noetic-tello-driver \
        python3-rosinstall \
        python3-pip \
        python3-yaml \
        python3-rospkg \
        python3-catkin-pkg

RUN pip3 install pyyaml
WORKDIR /ros
COPY . .
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc && update-alternatives --install /usr/bin/python python /usr/bin/python3 300

ENTRYPOINT ["/bin/bash", "-c", "./start.sh"]
