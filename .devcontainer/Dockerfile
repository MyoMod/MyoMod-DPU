FROM mcr.microsoft.com/devcontainers/base:ubuntu

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Find new gcc versions here: 
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
ENV GCC_VERSION=13.3.rel1
#ENV GCC_VERSION=12.3.rel1


#COPY version_vcpkg.env install_vcpkg.sh /opt/

# Install packages via apt
RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    #
    # Install packages for C++ development
    && apt-get -y install --no-install-recommends curl xz-utils x11-apps xauth\
    cmake ninja-build \
    #
    # [Optional] Update UID/GID if needed
    && if [ "$USER_GID" != "1000" ] || [ "$USER_UID" != "1000" ]; then \
    groupmod --gid $USER_GID $USERNAME \
    && usermod --uid $USER_UID --gid $USER_GID $USERNAME \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME; \
    fi \
    #
    # Install JLink
    && wget -q --post-data 'accept_license_agreement=accepted&non_emb_ctr=confirmed' https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb -O /opt/JLink_Linux_x86_64.deb \
    && printf '#!/bin/bash \n echo not running udevadm "$@"' > /usr/bin/udevadm && chmod +x /usr/bin/udevadm \
    && apt update && apt upgrade -y && apt install -y /opt/JLink_Linux_x86_64.deb \
    && rm -f /opt/JLink_Linux_x86_64.deb \
    #
    # Install ozone
    && wget -q https://www.segger.com/downloads/jlink/Ozone_Linux_V334_x86_64.deb \
    && apt install -y libxrandr-dev ./Ozone_Linux_V334_x86_64.deb \
    && rm -f Ozone_Linux_V334_x86_64.deb
    #
    # Clean up
    #&& apt-get autoremove -y \
    #&& apt-get clean -y \
    #&& rm -rf /var/lib/apt/lists/*

# Install GCC
WORKDIR /opt/
RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu/${GCC_VERSION}/binrel/arm-gnu-toolchain-${GCC_VERSION}-x86_64-arm-none-eabi.tar.xz | tar -xJ
#rename the folder
RUN mv /opt/arm-gnu-toolchain-${GCC_VERSION}-x86_64-arm-none-eabi /opt/arm-none-eabi-gcc-${GCC_VERSION} && \
    ln -s /opt/arm-none-eabi-gcc-${GCC_VERSION} /opt/arm-none-eabi-gcc
ENV PATH "$PATH:/opt/arm-none-eabi-gcc/bin"
# install missing dependencies
RUN apt-get update && apt-get install -y libncurses5 libncurses5-dev libncursesw5 libncursesw5-dev

# Enable pretty printing for gdb
RUN     apt install -y software-properties-common \
        && add-apt-repository ppa:deadsnakes/ppa -y \
        && apt update \
        && apt install -y python3.8 python3.8-distutils \
        #Install libstdc++6 for gdb pretty printing python scripts
        && apt install libstdc++6

# Tell gdb where to find the python scripts
COPY gdbinit /home/$USERNAME/.gdbinit