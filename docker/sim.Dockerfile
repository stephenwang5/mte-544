FROM ros:humble

RUN useradd -m -u 1000 default-user && \
    chsh -s /bin/bash default-user && \
    echo "default-user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    wget \
    tmux

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-turtlebot3*

COPY --chmod=755 docker/idle.bash /idle.bash
COPY docker/.tmux.conf /home/default-user/.tmux.conf
COPY docker/ros-env.bash /ros-env.bash
RUN cat /ros-env.bash >> /home/default-user/.bashrc

USER default-user
RUN mkdir -p /home/default-user/ws/src

WORKDIR /home/default-user/ws
