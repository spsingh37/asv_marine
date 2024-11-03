FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y git wget python3-pip vim
RUN pip3 install setuptools==58.2.0

RUN git clone https://github.com/osrf/vrx.git
RUN git config --global user.email "suryasin@umich.edu"
RUN git config --global user.name "spsingh37"

# Add ROS setup to .bashrc
RUN echo "source /opt/ros/humble/setup.sh" >> /root/.bashrc
    
# Install required packages
RUN apt-get update && apt-get install -y git wget python3-pip vim lsb-release curl gnupg && \
    pip3 install setuptools==58.2.0 && \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-garden python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro

# Create a colcon workspace and clone the vrx repository
# RUN mkdir -p vrx_ws/src && \
#     cd vrx_ws/src && \
#     git clone https://github.com/osrf/vrx.git

RUN mkdir -p vrx_ws/src
# Copy the local vrx repository into the container
COPY vrx /vrx_ws/src/vrx
RUN chmod +x vrx_ws/src/vrx/vrx_gz/scripts/dynamic_tf_broadcaster.py
RUN chmod +x vrx_ws/src/vrx/vrx_gz/scripts/static_tf_broadcaster.py

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    cd vrx_ws/src/vrx/vrx_gz && \
    colcon build --packages-select vrx_gz && \
    cd ../../.. && \
    colcon build --merge-install

# Install drivers and dependencies for PS4 connection
RUN git clone https://github.com/naoki-mizuno/ds4drv --branch devel && \
    cd ds4drv && \
    mkdir -p ~/.local/lib/python3.10/site-packages && \
    python3 setup.py install --prefix ~/.local || true && \
    pip install evdev && \
    apt -y install udev && \
    cp udev/50-ds4drv.rules /etc/udev/rules.d

RUN apt-get install -y tmux
RUN apt-get install -y ros-humble-joy-teleop
RUN pip install pyproj
RUN pip install scikit-learn

# Copy the entrypoint script into the container
COPY entrypoint.sh entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x entrypoint.sh

# Set the entrypoint
# ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
RUN ./entrypoint.sh

