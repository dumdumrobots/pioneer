FROM osrf/ros:humble-desktop-full

RUN apt-get update \
    && apt-get install -y nano \
    && rm -rf /var/lib/apt/lists/* 

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \ 
    && apt-get install -y net-tools \
    && apt-get install -y doxygen

RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN chmod 777 -R /dev/
# Create ROS-Workspace

RUN mkdir -p /home/$USERNAME/pioneer_ws/src

WORKDIR /home/$USERNAME/pioneer_ws

COPY src src

WORKDIR /home/$USERNAME/pioneer_ws/src/AriaCoda

RUN make  \
    && make install

WORKDIR /home/$USERNAME/pioneer_ws

ENV LD_LIBRARY_PATH ~/pioneer_ws/src/AriaCoda/lib

RUN rosdep install -i --from-path src --rosdistro humble -y \
    && . /opt/ros/humble/setup.sh \
    && colcon build \
    && . install/setup.sh


COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]