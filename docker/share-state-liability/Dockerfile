FROM ros

RUN apt-get update \
    && apt-get install -y git wget sudo syslinux libjpeg-dev libcairo-dev npm ros-kinetic-rosbridge-suite
RUN ln -s /usr/bin/nodejs /usr/bin/node
RUN useradd -m -g users ros
RUN echo "ros ALL=(ALL) NOPASSWD: /usr/bin/apt-get" >> /etc/sudoers

USER ros
RUN rosdep update

RUN cd ~ && git clone --recursive https://github.com/airalab/aira-IoT \
    && rm -rf aira-IoT/src/dron_ros_tutorial \
    && cd aira-IoT/src \
    && . /opt/ros/kinetic/setup.sh && catkin_init_workspace

RUN cd ~/aira-IoT && . /opt/ros/kinetic/setup.sh \
    && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y \
    && catkin_make

RUN cd ~/aira-IoT/src/aira_ros_bridge/aira_ros_bridge && npm install

RUN cd ~ && wget --no-check-certificate -O - -q https://dist.ipfs.io/go-ipfs/v0.4.8/go-ipfs_v0.4.8_linux-amd64.tar.gz | tar xzv go-ipfs/ipfs && mv go-ipfs/ipfs /usr/local/bin && rmdir go-ipfs

VOLUME ["/home/ros/state"]
WORKDIR /home/ros

ADD ./docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
