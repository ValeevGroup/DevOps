ARG ubuntuImage
FROM ${ubuntuImage}

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y locales && \
    locale-gen "en_US.UTF-8" && \
    update-locale LANG=en_US.UTF-8

ENV LANGUAGE en_US:en
ENV LANG en_US.UTF-8 
ENV LC_ALL en_US.UTF-8

# NB to enable nvidia docker runtime *with* --runtime nvidia
ENV NVIDIA_VISIBLE_DEVICES all

RUN mkdir -p /home/ValeevGroup/
COPY ./Makefile /home/ValeevGroup/
WORKDIR /home/ValeevGroup/

RUN DEBIAN_FRONTEND=noninteractive \
    apt-get -yq update && \
    apt-get -yq install make wget curl

# main, cmake, clang
RUN make install/main && \
    make install/cmake && \
    make install/clang && \
    apt-get clean

# Add MKL repo, download, but don't install
RUN make apt-add-repository/intel-mkl && \
    make download/intel-mkl
# RUN make /home/ValeevGroup/ install/intel-mkl

# libint
RUN make install/libint2 && \
    rm -fr /home/ValeevGroup/libint2

