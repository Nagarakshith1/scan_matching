FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    ros-noetic-ackermann-msgs \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-joy \
    ros-noetic-map-server \
    ros-noetic-xacro \
    ros-noetic-cv-bridge \
    git

ENV QT_X11_NO_MITSHM 1

RUN mkdir -p /home/ros/catkin_ws/src

RUN git clone https://github.com/f1tenth/f1tenth_simulator.git /home/ros/catkin_ws/src/f1tenth_simulator

RUN git clone https://github.com/Nagarakshith1/scan_matching.git /home/ros/catkin_ws/src/scan_matching

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/ros/catkin_ws; catkin_make'

ENTRYPOINT ["/bin/bash", "-c" , ". /opt/ros/noetic/setup.bash && . /home/ros/catkin_ws/devel/setup.bash && roslaunch scan_matching scan_matching.launch"]