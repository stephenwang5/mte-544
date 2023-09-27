FROM ros:humble

RUN useradd -m -u 1000 default-user && \
    chsh -s /bin/bash default-user && \
    echo "default-user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    wget \
    tmux

RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get install -y --no-install-recommends \
    gazebo11

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-moveit

COPY --chmod=755 docker/idle.bash /idle.bash
COPY docker/.tmux.conf /home/default-user/.tmux.conf
COPY docker/ros-env.bash /ros-env.bash
RUN cat /ros-env.bash >> /home/default-user/.bashrc

USER default-user
RUN mkdir -p /home/default-user/ws/src

WORKDIR /home/default-user/ws
COPY catkin_ws src
RUN rosdep update && \
    rosdep install -y -i --from-paths src
