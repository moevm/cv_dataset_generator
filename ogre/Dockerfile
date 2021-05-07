FROM ubuntu:20.04

# Depencencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    libgles2-mesa-dev \
    libxrandr-dev \
    zlib1g \
    libsdl2-dev \
    libxt-dev \
    libopencv-dev python3-opencv \
    libyaml-cpp-dev \
    wget \
    && apt-get clean

# Retrieving OGRE sources
RUN mkdir /src && \
    cd /src && \
    wget https://github.com/OGRECave/ogre/archive/refs/tags/v1.12.12.tar.gz && \
    tar xf v1.12.12.tar.gz && \
    mv ogre-1.12.12 ogre && \
    rm v1.12.12.tar.gz

# Building OGRE
RUN cd /src/ogre \
    && mkdir build \
    && cd build \
    && cmake .. \
    && cmake --build . --config release -j 3 \
    && make install

# Building the app
COPY . /src/SimpleScene/
WORKDIR /src/SimpleScene

RUN cmake CMakeLists.txt && make -j 4

ENTRYPOINT ["./SimpleScene"]