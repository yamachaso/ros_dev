FROM yamachaso/ros_melodic_base:latest

# BUILD_USERはdocker-compose.yml内で指定
ARG BUILD_USER
ENV USER ${BUILD_USER}
ENV HOME /home/${USER}
ENV ROS_WORKSPACE ${HOME}/catkin_ws
ENV PATH=${HOME}/.local/bin:${PATH}

USER root
RUN usermod -l ${USER} appuser
RUN usermod -d /home/${USER} -m ${USER}
RUN usermod -c ${USER} ${USER}
RUN sed -i s/appuser/${USER}/ ${HOME}/.bashrc
RUN rm -R -f /home/appuser

USER ${USER}
WORKDIR ${ROS_WORKSPACE}

# エイリアス
RUN echo 'alias ccp="catkin_create_pkg"' >> ${HOME}/.bashrc
RUN echo 'alias cb="catkin build"' >> ${HOME}/.bashrc
RUN echo 'alias rl="roslaunch"' >> ${HOME}/.bashrc
