FROM continuumio/anaconda3:latest

RUN apt-get update
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update -y > /dev/null
RUN apt-get install -y apt-utils > /dev/null
RUN apt-get install -y build-essential > /dev/null
RUN apt-get install -y pkg-config > /dev/null
RUN apt-get install -y libeigen3-dev > /dev/null
RUN apt-get install -y libboost-all-dev
RUN wget https://github.com/Kitware/CMake/releases/download/v3.19.0/cmake-3.19.0-Linux-x86_64.sh \
      -q -O /tmp/cmake-install.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && mkdir /usr/bin/cmake \
      && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake \
      && rm /tmp/cmake-install.sh
ENV PATH="/usr/bin/cmake/bin:${PATH}"
RUN apt-get install -y gcovr > /dev/null
