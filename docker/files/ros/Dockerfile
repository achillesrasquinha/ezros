FROM ghcr.io/alpine-ros/alpine-ros:noetic-ros-core

ARG HOME=/home/ros \
    ROS_VNC_PASSWORD=ros

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    PYTHONUNBUFFERED=1 \
    DISPLAY=:0.0 \
    TERM=xterm \
    RESOLUTION=1920x1080x24 \
    ROS_VNC_PORT=5900 \
    ROS_HOST=ros \
    ROS_PORT=8080 \
    ROS_SOCKET_PORT=3000 \
    ROS_GROUP=ros \
    ROS_USER=ros \
    ROS_BRIDGE_LOCATION="/dev/tty.Bridge-Port" \
    ROS_BRIDGE_TCP_HOST=host.docker.internal \
    ROS_BRIDGE_TCP_PORT=7777 \
    ROS_PACKAGE_PATH=/home/ros/catkin_ws/src \
    DPYTHON_EXECUTABLE=/usr/bin/python3 \
    WEBSOCKIFY_VERSION=0.10.0 \
    ROS_DISTRO=noetic \
    ROS_CORE_PORT=13111 \
    ROS_MASTER_URI=http://ros:13111

RUN set -e && \
    addgroup -S ${ROS_GROUP} && \
    adduser  -h ${HOME} -S ${ROS_USER} -G ${ROS_GROUP} && \
    apk update && \
    apk add --no-cache \
        bash \
        git \
        nano \
        x11vnc \
        xvfb \
        xfce4 \
        xterm \
        faenza-icon-theme \
        python3 \
        py3-pip \
        socat \
        make \
        gcc \
        g++ \
        sudo \
        ros-${ROS_DISTRO}-turtlesim \
        openrc \
        lightdm \
        dbus && \
    dbus-uuidgen --ensure=/etc/machine-id && \
    if [ ! -e /usr/bin/python ]; then ln -sf python3 /usr/bin/python; fi && \
    if [ ! -e /usr/bin/pip ]; then ln -sf pip3 /usr/bin/pip; fi && \
    pip install --upgrade pip wheel "setuptools<50" && \
    pip install \
        pyserial \
        https://github.com/achillesrasquinha/bpyutils.git@develop#egg=bpyutils && \
    echo "$ROS_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    git clone --depth 1 https://github.com/novnc/noVNC.git /noVNC && \
    git clone --depth 1 https://github.com/novnc/websockify /noVNC/utils/websockify && \
    rm -rf /noVNC/.git && \
    rm -rf /noVNC/utils/websockify/.git && \
    # apk del git && \
    sed -i -- "s/ps -p/ps -o pid | grep/g" /noVNC/utils/novnc_proxy && \
    cp /noVNC/vnc.html /noVNC/index.html && \
    # change default shell from ash to bash
    sed -i -e "s/bin\/ash/bin\/bash/" /etc/passwd && \
    echo "source $HOME/catkin_ws/devel/setup.sh" >> $HOME/.bashrc && \
    rc-update add dbus && \
    rc-update add lightdm

USER ${ROS_USER}

WORKDIR ${HOME}

RUN mkdir -p ${HOME}/.vnc && \
    x11vnc -storepasswd ${ROS_VNC_PASSWORD} ${HOME}/.vnc/passwd && \
    touch ${HOME}/.Xauthority

COPY --chown=${ROS_GROUP}:${ROS_USER} ./start /start

RUN chmod +x /start && \
    mkdir -p ${HOME}/catkin_ws/src && \
    sudo chown ${ROS_GROUP}:${ROS_USER} ${HOME}/catkin_ws

CMD [ "/start" ]