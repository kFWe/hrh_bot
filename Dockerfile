FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ENV SHELL=/bin/bash
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    bash \
    git \
    cppcheck \
    libmodbus-dev \
    cccc \
    wget \
    zip \
    unzip \
    jq \
    nano \
    build-essential \
    libssl-dev \
    usbutils \
    gdb-multiarch \
    automake \
    autoconf \
    texinfo \
    libtool \
    libftdi-dev \
    libusb-1.0-0-dev \
    pkg-config \
    cmake

# GDB dependencies
RUN apt-get update && apt-get install -y \
    libncurses5 \
    libncurses5-dev \
    libncursesw5 \
    libncursesw5-dev \
    python3-dev \
    python3 \
    python3-venv \
    python3-pip \
    expat

# JLINK gui dependencies
RUN apt-get update && apt-get install -y \
    libxrandr-dev \
    libxfixes-dev \
    libxcursor-dev

RUN useradd -ms /bin/bash --user-group --groups dialout vscode

# Switch user back to root
USER root
ARG TOOLCHAIN_DIR="/opt/toolchain"

# Copy bootstrap script and patch files into toolchain directory
COPY ./libs/platforms/scripts/bootstrap.sh /

# Run bootstrap script
RUN bash -c "/bootstrap.sh --gcc --openocd --pico_sdk --jlink --jsystemview"
