FROM ghcr.io/ascendntnu/perception:foxy-base as base
SHELL ["/bin/bash", "-c"]
# Run a bash shell unless something else is specified
CMD ["bash"]


# Deploy image contains a ready-built version of the workspace
FROM base as deploy
# Copy and build source
RUN mkdir /ws
WORKDIR /ws
COPY ./src/ /ws/src/
RUN . /opt/ros/foxy/setup.sh && colcon build
# Setup entrypont with access to ros2 and the built workspace 
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]


# Used as devcontainer in vscode
FROM base as devcontainer
ARG WORKSPACE
# Update .bashrc files to make terminal user-friendly
# Use both root and ros users to ensure rootless/root docker compatibility
COPY ./docker/.bashrc /root/
COPY ./docker/.bashrc /home/ros/
# Automatically source the workspace
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc
RUN chown -R 1000:1000 /home/ros
RUN chown -R 0:0 /root
# Link to correct clang version to avoid warning in VSCode
RUN update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-13 0
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-13 0