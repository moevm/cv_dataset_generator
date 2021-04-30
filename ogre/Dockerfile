FROM ubuntu:20.04

# Depencencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    libgles2-mesa-dev \
    libxrandr-dev \
    git \
    zlib1g \
    libsdl2-dev \
    libxt-dev \
    libopencv-dev python3-opencv \
    libyaml-cpp-dev \
    && apt-get clean

# Retrieving OGRE sources
RUN git clone https://github.com/OGRECave/ogre.git /src/ogre 

# Building OGRE
RUN cd /src/ogre \
    && mkdir build \
    && cd build \
    && cmake .. \
    && cmake --build . --config release \
    && make install

# Building the app
COPY . /src/SimpleScene/
WORKDIR /src/SimpleScene

RUN cmake CMakeLists.txt && make -j

ENTRYPOINT ["./SimpleScene"]