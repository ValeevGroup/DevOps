ubuntu_codename ?= $(shell lsb_release -sc)
libint2_version ?= 2.7.2
clang_version ?= 9
cuda_version ?= 11.6
cuda_driver_version ?= 510  # must match the deriver on the host

export DEBIAN_FRONTEND=noninteractive

ubuntu_release = $(shell lsb_release -sr)

cuda_repo = https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(subst .,,${ubuntu_release})/x86_64
cuda_tag := $(subst .,-,${cuda_version})
cuda_packages := cuda-minimal-build-${cuda_tag} \
                 cuda-nvtx-${cuda_tag} cuda-nvprof-${cuda_tag} \
                 libcublas-${cuda_tag} libcublas-dev-${cuda_tag}

intel_mkl_version ?= 2020.4-304
intel_mkl := intel-mkl-\*-${intel_mkl_version}

install/main: install/libboost-latest
	apt-get update
	apt-get -yq install \
          lsb-release coreutils sudo bash-completion \
          apt-transport-https software-properties-common ca-certificates gnupg \
          linux-tools-common time pciutils \
          build-essential wget curl \
          git make ninja-build \
          gcc g++ gfortran gdb valgrind \
          libeigen3-dev \
          libblas-dev liblapack-dev liblapacke-dev \
          libunwind-dev libtbb-dev libomp-dev \
          libopenmpi-dev openmpi-bin libscalapack-openmpi-dev \
          python3 python3-pip python3-numpy python3-dev python3-pytest \
          vim emacs-nox

install/docker:
	sudo apt-get install docker.io

install/g++-%:
	apt-get -yq install gcc-$* g++-$* gfortran-$*

install/add-apt-repository:
	apt-get update
	apt-get -yq install software-properties-common

install/libboost-latest: install/add-apt-repository
	add-apt-repository ppa:mhier/libboost-latest  # need boost 1.68+, but ubuntu 18 only provides boost 1.65
	apt-get -yq install libboost1.74-dev

# CMake
install/cmake:
	wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add -
	apt-add-repository "deb https://apt.kitware.com/ubuntu/ ${ubuntu_codename} main"
	apt-get -yq update
	apt-get -yq install cmake cmake-curses-gui


# LLVM
install/clang-%:
	wget https://apt.llvm.org/llvm-snapshot.gpg.key -O - | apt-key add -
	add-apt-repository "deb http://apt.llvm.org/${ubuntu_codename}/ llvm-toolchain-${ubuntu_codename}-$* main"
	apt-get -yq update
	apt-get -yq install clang-$*

install/clang: install/clang-${clang_version}


# Intel
apt-add-repository/intel-mkl:
	wget -O - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB | apt-key add -
	apt-add-repository "deb https://apt.repos.intel.com/mkl all main"
	apt-get -yq update

download/intel-mkl:
	apt-get install -yq --download-only ${intel_mkl}

install/intel-mkl:
	apt-get -yq install ${intel_mkl}
	rm -rf /opt/intel/mkl
	ln -s /opt/intel/compilers_and_libraries_$(subst -,.,${intel_mkl_version})/linux/mkl /opt/intel/mkl
	test -x /opt/intel/mkl/bin/mklvars.sh

# CUDA, host-only stuff, nothing to be done in the container
install/nvidia-docker:
	test -x `which curl`
	curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
	curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu${ubuntu_release}/nvidia-docker.list \
          | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
	sudo apt-get update
	sudo apt-get install -y nvidia-docker2

# Libint2
libint2:
	wget --no-verbose https://github.com/evaleev/libint/releases/download/v${libint2_version}/libint-${libint2_version}-mpqc4.tgz
	#wget --no-verbose -O libint-${libint2_version}.tgz "https://drive.google.com/uc?export=download&id=159sKKdUdi8WBN7JerG2XvwuHo8EAKNkH"
	tar xzf libint-${libint2_version}-mpqc4.tgz
	mkdir libint2
	cd libint2 && cmake ../libint-${libint2_version} -DCMAKE_POSITION_INDEPENDENT_CODE=ON
	cd libint2 && make -j16

.PHONY: install/libint2
install/libint2: libint2
	make -C libint2 install
