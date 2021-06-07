# Created by Sebastian Maierhofer
FROM ubuntu:20.04

WORKDIR /commonroad

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential git pkg-config wget libomp-dev \
        gcovr doxygen graphviz cppcheck \
        libeigen3-dev libboost-all-dev uuid-dev \
        libcgal-dev && \
    rm -rf /var/lib/apt/lists/*

ARG CMAKE_VERSION=3.19.0
RUN apt-get update && apt-get upgrade -y ca-certificates && wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh" \
      -q -O /tmp/cmake-install.sh && \
      chmod u+x /tmp/cmake-install.sh && \
      mkdir /opt/cmake && \
      /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake && \
      rm /tmp/cmake-install.sh
ENV PATH="/opt/cmake/bin:${PATH}"

# Install clang and extra clang tools
ARG CLANG_VERSION=10
ARG CLANG_FORMAT_VERSION=10
ARG CLANG_TIDY_VERSION=10
RUN apt-get update && \
    apt-get install -y \
        clang-${CLANG_VERSION} \
        clang-format-${CLANG_FORMAT_VERSION} \
        clang-tidy-${CLANG_TIDY_VERSION} && \
    rm -rf /var/lib/apt/lists/*

# Setup environment variables, PATH first then C and C++ compilers to clang
ENV PATH="$PATH:/usr/lib/llvm-${COMPILER_VERSION}/bin/"
# Append llvm directory to path, that we can use clang-format instead of clang-format-<some-version>
ENV PATH="$PATH:/usr/lib/llvm-${CLANG_FORMAT_VERSION}/bin/"
# Append llvm directory to path, that we can use clang-tidyinstead of clang-tidy-<some-version>
ENV PATH="$PATH:/usr/lib/llvm-${CLANG_TIDY_VERSION}/bin/"


ARG PYTHON_VERSION=3.8
RUN wget https://repo.anaconda.com/archive/Anaconda3-2020.11-Linux-x86_64.sh -O /commonroad/anaconda.sh && \
    bash /commonroad/anaconda.sh -b && \
    rm /commonroad/anaconda.sh
ENV PATH /root/anaconda3/bin:$PATH

# Updating Anaconda packages
RUN conda update conda && \
    conda update anaconda && \
    conda update --all && \
    conda create --name commonroad python=${PYTHON_VERSION}
ENV CONDA_DEFAULT_ENV $commonroad

RUN mkdir -p -m 0600 ~/.ssh && touch ~/.ssh/known_hosts && ssh-keyscan gitlab.lrz.de >> ~/.ssh/known_hosts

RUN --mount=type=ssh \
    git clone git@gitlab.lrz.de:cps/commonroad-drivability-checker.git && \
    cd commonroad-drivability-checker && \
    git checkout feature_cmake_export && \
    git submodule update --init

RUN mkdir dist && \
    cd commonroad-drivability-checker && \
    mkdir build-cpp && cd build-cpp && \
    cmake -DCMAKE_INSTALL_PREFIX=/commonroad/dist -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && \
    make -j4 && \
    make install && \
    cd .. && rm -rf build-cpp

RUN /bin/bash -c "source activate commonroad"

RUN cd commonroad-drivability-checker && \
    python setup.py bdist_wheel && \
    python setup.py install
