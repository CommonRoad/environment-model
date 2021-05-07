FROM ubuntu:20.04

RUN apt-get update
RUN apt-get upgrade -y
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update -y > /dev/null
RUN apt-get install -y apt-utils > /dev/null
RUN apt-get install -y build-essential > /dev/null
RUN apt-get install -y pkg-config > /dev/null
RUN apt-get install -y libeigen3-dev > /dev/null
RUN apt-get install -y libboost-all-dev > /dev/null
RUN apt-get install -y wget > /dev/null
RUN wget https://github.com/Kitware/CMake/releases/download/v3.19.0/cmake-3.19.0-Linux-x86_64.sh \
      -q -O /tmp/cmake-install.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && mkdir /usr/bin/cmake \
      && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake \
      && rm /tmp/cmake-install.sh
ENV PATH="/usr/bin/cmake/bin:${PATH}"
RUN apt-get install -y gcovr > /dev/null
RUN apt-get install -y doxygen > /dev/null
RUN apt-get install -y uuid-dev > /dev/null
RUN apt-get install -y libc++-dev > /dev/null
RUN apt-get install -y libc++abi-dev > /dev/null
RUN apt-get install -y libomp-dev > /dev/null


# Install the compiler, libc++, libc++-abi and OpenMP
ARG COMPILER_VERSION
RUN apt-get install -y clang-${COMPILER_VERSION} > /dev/null
# Setup enviorment variables, PATH first then C and C++ compilers to clang
ENV PATH="$PATH:/usr/lib/llvm-${COMPILER_VERSION}/bin/"

ARG CLANG_FORMAT_VERSION=10
RUN apt install git clang-format-${CLANG_FORMAT_VERSION} -y \
  && rm -rf /var/lib/apt/lists/* > /dev/null
# Append llvm directory to path, that we can use clang-format instead of clang-format-<some-version>
ENV PATH="$PATH:/usr/lib/llvm-${CLANG_FORMAT_VERSION}/bin/"

ARG CLANG_TIDY_VERSION=10
RUN set -eux && apt update && apt upgrade -qqy && apt-get install -y clang-tidy \
  && rm -rf /var/lib/apt/lists/* > /dev/null
# Append llvm directory to path, that we can use clang-tidyinstead of clang-tidy-<some-version>
ENV PATH="$PATH:/usr/lib/llvm-${CLANG_TIDY_VERSION}/bin/"

#ARG PYTHON_VERSION=3.8
#RUN wget https://repo.anaconda.com/archive/Anaconda3-2020.11-Linux-x86_64.sh \
#    && bash Anaconda3-2020.11-Linux-x86_64.sh -y
#RUN conda create --name commonroad python=PYTHON_VERSION -y
#RUN conda activate commonroad
