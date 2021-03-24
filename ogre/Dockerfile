FROM ubuntu:18.04

# Depencencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libgles2-mesa-dev \
    libxrandr-dev \
    git \
    zlib1g \
    libsdl2-dev \
    libxt-dev \
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

RUN cmake CMakeLists.txt && make

ENTRYPOINT ["./SimpleScene"]