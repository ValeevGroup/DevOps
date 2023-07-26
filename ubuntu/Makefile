ubuntu_codename ?= $(shell lsb_release -sc)
ubuntu_release = $(shell lsb_release -sr)
libint2_version ?= 2.7.2
clang_version ?= 13
cuda_version ?= 12.1

export DEBIAN_FRONTEND=noninteractive

ifeq ($(origin gcc_version), undefined)
  ifeq ($ubuntu_codename, jammy)
	gcc_version = 13
  else
    gcc_version = 10
  endif
endif

cuda_repo = https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(subst .,,${ubuntu_release})/x86_64
cuda_tag := $(subst .,-,${cuda_version})
cuda_packages := cuda-minimal-build-${cuda_tag} \
                 cuda-nvtx-${cuda_tag} cuda-nvprof-${cuda_tag} \
                 libcublas-${cuda_tag} libcublas-dev-${cuda_tag}

intel_mkl_version ?= 2020.4-304
intel_mkl := intel-mkl-\*-${intel_mkl_version}

install/main: install/libboost-latest install/gcc-${gcc_version}
	apt-get update
	apt-get -yq install \
          lsb-release coreutils sudo bash-completion \
          apt-transport-https software-properties-common ca-certificates gnupg \
          linux-tools-common time pciutils \
          build-essential wget curl \
          git make ninja-build \
          gdb valgrind \
          libeigen3-dev \
          libblas-dev liblapack-dev liblapacke-dev \
          libunwind-dev libtbb-dev libomp-dev \
          libopenmpi-dev openmpi-bin libscalapack-openmpi-dev \
          python3 python3-pip python3-numpy python3-dev python3-pytest \
          vim emacs-nox ccache

install/docker:
	sudo apt-get install docker.io

install/gcc-%:
	apt-get -yq install gcc-$* g++-$* gfortran-$*

install/add-apt-repository:
	apt-get update
	apt-get -yq install software-properties-common

install/libboost-latest:
	apt-get -yq install libboost-all-dev

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
	apt-get -yq install clang-$* libomp-$*-dev

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

# CUDA
apt-add-repository/cuda:
	wget ${cuda_repo}/cuda-ubuntu$(subst .,,${ubuntu_release}).pin -O /etc/apt/preferences.d/cuda-repository-pin-600
	wget ${cuda_repo}/cuda-keyring_1.0-1_all.deb -O /tmp/cuda-keyring.deb
	sudo dpkg -i /tmp/cuda-keyring.deb
	rm /tmp/cuda-keyring.deb
	#apt-key adv --fetch-keys ${cuda_repo}/7fa2af80.pub
	add-apt-repository "deb ${cuda_repo} /"
	apt-get update

download/cuda:
	apt-get install -yq --download-only ${cuda_packages}

install/cuda: download/cuda
	apt-get install -yq ${cuda_packages}
	rm -fr /usr/local/cuda
	ln -s /usr/local/cuda-$(subst -,.,${cuda_tag}) /usr/local/cuda
	rm -fr /usr/local/bin/nvcc
	# NB /usr/local/cuda/bin/nvcc symlink breaks CMake
	#ln -s /usr/local/cuda/bin/nvcc /usr/local/bin/nvcc

install/nvidia-docker:
	test -x `which curl`
	curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
	curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu${ubuntu_release}/nvidia-docker.list \
          | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
	sudo apt-get update
	sudo apt-get install -y nvidia-docker2

# Libint2
libint2:
	wget --no-verbose https://github.com/evaleev/libint/releases/download/v${libint2_version}/libint-${libint2_version}-mpqc4-x2c.tgz
	tar xzf libint-${libint2_version}-mpqc4-x2c.tgz
	mkdir libint2
	cd libint2 && cmake ../libint-${libint2_version} -DCMAKE_POSITION_INDEPENDENT_CODE=ON
	cd libint2 && make -j16

.PHONY: install/libint2
install/libint2: libint2
	make -C libint2 install
