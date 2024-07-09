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
    && apt-get -y install --no-install-recommends curl xz-utils \
    cmake ninja-build \
    #
    # [Optional] Update UID/GID if needed
    && if [ "$USER_GID" != "1000" ] || [ "$USER_UID" != "1000" ]; then \
    groupmod --gid $USER_GID $USERNAME \
    && usermod --uid $USER_UID --gid $USER_GID $USERNAME \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME; \
    fi \
    #
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install GCC
WORKDIR /opt/
RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu/${GCC_VERSION}/binrel/arm-gnu-toolchain-${GCC_VERSION}-x86_64-arm-none-eabi.tar.xz | tar -xJ
#rename the folder
RUN mv /opt/arm-gnu-toolchain-${GCC_VERSION}-x86_64-arm-none-eabi /opt/arm-none-eabi-gcc-${GCC_VERSION} && \
    ln -s /opt/arm-none-eabi-gcc-${GCC_VERSION} /opt/arm-none-eabi-gcc
ENV PATH "$PATH:/opt/arm-none-eabi-gcc/bin"

# install vcpkg
# ENV PATH=${NINJA_ROOT}:${CMAKE_ROOT}/bin:${VCPKG_ROOT}:${PATH}
# ENV VCPKG_FORCE_SYSTEM_BINARIES=1

# COPY version_vcpkg.env install_vcpkg.sh /opt/

# # Download, build and install tools
# RUN cd /opt && \
#     chmod a+x install_vcpkg.sh && \
#     ./install_vcpkg.sh ${VCPKG_ROOT} ${NINJA_ROOT} ${CMAKE_ROOT} && \
#     rm install_vcpkg.sh && rm version_vcpkg.env