FROM humble:dev

RUN apt-get update && apt-get install -y \
        curl \
        putty \
        # ros
        ros-humble-ros2-control \
        ros-humble-turtlebot3-teleop \
        ros-humble-gazebo-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard \
        joystick \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout user

USER user

RUN python3 -m pip install -U platformio
COPY .devcontainer/requirements.txt /tmp
RUN pip install --no-cache-dir -r /tmp/requirements.txt
# RUN /home/user/.local/bin/pio/pio platform install espressif32