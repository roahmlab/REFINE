# Copyright 2019 - 2022 The MathWorks, Inc.

# To specify which MATLAB release to install in the container, edit the value of the MATLAB_RELEASE argument.
# Use lower case to specify the release, for example: ARG MATLAB_RELEASE=r2021b
ARG MATLAB_RELEASE=r2022a

# When you start the build stage, this Dockerfile by default uses the Ubuntu-based matlab-deps image.
# To check the available matlab-deps images, see: https://hub.docker.com/r/mathworks/matlab-deps
FROM mathworks/matlab-deps:${MATLAB_RELEASE}

# Declare the global argument to use at the current build stage
ARG MATLAB_RELEASE

# Install mpm dependencies
RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && \
    apt-get install --no-install-recommends --yes \
        wget \
        unzip \
        ca-certificates && \
    apt-get clean && apt-get autoremove

# Run mpm to install MATLAB in the target location and delete the mpm installation afterwards.
# If mpm fails to install successfully then output the logfile to the terminal, otherwise cleanup.
RUN wget -q https://www.mathworks.com/mpm/glnxa64/mpm && \ 
    chmod +x mpm && \
    ./mpm install \
        --release=${MATLAB_RELEASE} \
        --destination=/opt/matlab \
        --products MATLAB ROS_Toolbox Mapping_Toolbox Optimization_Toolbox Phased_Array_System_Toolbox DSP_System_Toolbox Signal_Processing_Toolbox || \
    (echo "MPM Installation Failure. See below for more information:" && cat /tmp/mathworks_root.log && false) && \
    rm -f mpm /tmp/mathworks_root.log && \
    ln -s /opt/matlab/bin/matlab /usr/local/bin/matlab

# Add "matlab" user and grant sudo permission.
RUN adduser --shell /bin/bash --disabled-password --gecos "" matlab && \
    echo "matlab ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/matlab && \
    chmod 0440 /etc/sudoers.d/matlab

# One of the following 2 ways of configuring the license server to use must be
# uncommented.

ARG LICENSE_SERVER
# Specify the host and port of the machine that serves the network licenses 
# if you want to bind in the license info as an environment variable. This 
# is the preferred option for licensing. It is either possible to build with 
# something like --build-arg LICENSE_SERVER=27000@MyServerName, alternatively
# you could specify the license server directly using
#       ENV MLM_LICENSE_FILE=27000@flexlm-server-name
ENV MLM_LICENSE_FILE=$LICENSE_SERVER

# Alternatively you can put a license file into the container.
# You should fill this file out with the details of the license 
# server you want to use and uncomment the following line.
# COPY network.lic /opt/matlab/licenses/

# The following environment variables allow MathWorks to understand how this MathWorks 
# product (MATLAB Dockerfile) is being used. This information helps us make MATLAB even better. 
# Your content, and information about the content within your files, is not shared with MathWorks. 
# To opt out of this service, delete the environment variables defined in the following line. 
# See the Help Make MATLAB Even Better section in the accompanying README to learn more: 
# https://github.com/mathworks-ref-arch/matlab-dockerfile#help-make-matlab-even-better
ENV MW_DDUX_FORCE_ENABLE=true MW_CONTEXT_TAGS=MATLAB:DOCKERFILE:V1

# Set user and work directory
#USER matlab
#WORKDIR /home/matlab
#ENTRYPOINT ["matlab"]
#CMD [""]

# Fix matlab warning
RUN apt-get install -yq libcanberra-gtk-module libcanberra-gtk3-module




#
# ROS Installation
#

# Upgrade beforehand
RUN apt-get update -yq
RUN apt-get upgrade -yq

# Basic utilities
RUN apt-get install -yq git 
RUN apt-get install -yq wget 
RUN apt-get install -yq vim


# Install ROS
RUN apt-get install -yq lsb-release
RUN apt-get install -yq gnupg
RUN apt-get install -yq curl
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tzdata
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update -yq
RUN apt-get install -yq ros-noetic-ros-base
RUN apt-get install -yq python3-catkin-tools
RUN apt-get install -yq ros-noetic-joy
RUN apt-get install -yq ros-noetic-tf*
RUN apt-get install -yq ros-noetic-tf2*
RUN apt-get install -yq ros-noetic-jsk-recognition-msgs
RUN apt-get install -yq libxmlrpcpp-dev librosconsole-dev
RUN apt-get install -yq g++-9
RUN apt-get install -yq clang++-12
RUN apt-get install -yq clang-format-12
RUN apt-get install -yq doxygen
RUN apt-get install -yq ripgrep

# Install newer cmake version
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.5/cmake-3.22.5-linux-x86_64.sh \
      -q -O /tmp/cmake-install.sh \
    && chmod u+x /tmp/cmake-install.sh \
    && mkdir -p /usr/bin/cmake_v3.22.5 \
    && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake_v3.22.5 \
    && rm /tmp/cmake-install.sh
ENV PATH="/usr/bin/cmake/bin:${PATH}"

RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update -yq
RUN apt-get install -yq --no-install-recommends ros-foxy-ros-core
RUN apt-get install -yq ros-foxy-demo-nodes-cpp-native
RUN apt-get install -yq --no-install-recommends ros-foxy-desktop

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy

RUN apt-get install -yq software-properties-common
RUN apt-get update
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get -yq install python3.9 python3.9-venv
RUN apt-get install -yq python3-empy












#
# Install IPOPT
#
COPY data/coinhsl-2021.05.05.tar.gz /tmp/coinhsl-2021.05.05.tar.gz
#COPY setup-ipopt-and-hsl.sh /tmp/setup-ipopt-and-hsl.sh
#RUN /tmp/setup-ipopt-and-hsl.sh
#RUN chmod -R 777 /installs

# Ipopt Version to install, note that the specified version
# must also have a corresponding release tag in Ipopt's git repo
ARG IPOPT_VERS=3.14.4

# Directory to make new folder in
ARG TOP_LEVEL_SRC_DIR=/installs

# Location of CoinHSL tar.gz file
ARG COINHSL_TAR_GZ_DIR=/tmp

# Name of CoinHSL file without the extension
ARG COINHSL_TAR_GZ_FBASE_NAME=coinhsl-2021.05.05

# -----------------------------------------------------------------------------
# The following directories will be created
#   (parent) TOP_LEVEL_SRC_DIR/Ipopt
#   (hsl)    TOP_LEVEL_SRC_DIR/Ipopt/ThirdParty-HSL
#   (ipopt)  TOP_LEVEL_SRC_DIR/Ipopt/Ipopt-$IPOPT_VERS
# 
# HSL will be copied from COINHSL_TAR_GZ_FPATH/COINHSL_TAR_GZ_FBASE_NAME.tar.gz
#   and extracted to
#   to TOP_LEVEL_SRC_DIR/Ipopt/ThirdParty-HSL/coinhsl
# -----------------------------------------------------------------------------

# Setup directories and filepaths
ARG IPOPT_PARENT_DIR=$TOP_LEVEL_SRC_DIR/Ipopt
ARG IPOPT_WITH_VERS=Ipopt-$IPOPT_VERS
ARG COINHSL_TAR_GZ_FNAME=$COINHSL_TAR_GZ_FBASE_NAME.tar.gz
ARG COINHSL_TAR_GZ_FPATH=$COINHSL_TAR_GZ_DIR/$COINHSL_TAR_GZ_FNAME
RUN apt-get install -yq \
	g++ \
	gcc \
	gfortran \
	git \
	libatlas-base-dev \
	liblapack-dev \
	libmetis-dev \
	patch \
	pkg-config \
	unzip \
	wget

RUN mkdir -p $IPOPT_PARENT_DIR
WORKDIR $IPOPT_PARENT_DIR

# -----------
# Install HSL
# -----------

# Clone the repo
RUN git clone https://www.github.com/coin-or-tools/ThirdParty-HSL.git
WORKDIR ThirdParty-HSL

# Extract HSL to local directory
RUN tar -xvzf $COINHSL_TAR_GZ_FPATH

# Rename extracted directory for installation purposes
RUN mv $COINHSL_TAR_GZ_FBASE_NAME coinhsl

RUN ./configure && \
  make -j && \
  make install

# -------------
# Install IPOPT
# -------------

# Clone IPOPT and checkout relevant version
RUN git clone https://www.github.com/coin-or/Ipopt.git $IPOPT_WITH_VERS
WORKDIR $IPOPT_WITH_VERS
RUN git checkout releases/$IPOPT_VERS

# Make a build directory and configure
RUN mkdir -p build
WORKDIR build

RUN ../configure --with-hsl --with-lapack --with-lapack-lflags="-lf77blas -lcblas -latlas -llapack" && \
  make && \
  make test && \
  make install

RUN sudo apt install -yq libomp-dev

# ----------------
# Install valgrind
# ----------------

# Download the valgrind package manually, install with no-deps, and remove
RUN wget http://archive.ubuntu.com/ubuntu/pool/main/v/valgrind/valgrind_3.15.0-1ubuntu9.1_amd64.deb && \
  dpkg --ignore-depends=libc6-dbg -i valgrind_3.15.0-1ubuntu9.1_amd64.deb && \
  rm -f valgrind_3.15.0-1ubuntu9.1_amd64.deb


# ------------
# Install cuda
# ------------

# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
# RUN dpkg -i cuda-keyring_1.0-1_all.deb
# RUN apt-get -yq update
# RUN apt-get -yq install cuda
# ENV PATH /usr/local/cuda-11.8/bin${PATH:+:${PATH}}


WORKDIR /simulator
#RUN /bin/bash -c "cd /installs/cpp-opt/src && source /opt/ros/noetic/setup.bash && catkin build"
