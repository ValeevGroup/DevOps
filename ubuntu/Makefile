ubuntu_codename ?= $(shell cat /etc/os-release | grep UBUNTU_CODENAME= | sed 's/UBUNTU_CODENAME=//g' )
ubuntu_release ?= $(shell cat /etc/os-release | grep VERSION_ID= | sed 's/VERSION_ID=//g' | sed 's/"//g' )
libint2_version ?= 2.10.2
libxc_version ?= 6.2.0
cuda_version ?= 12.6

export DEBIAN_FRONTEND=noninteractive

cuda_repo = https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(subst .,,${ubuntu_release})/x86_64
cuda_tag := $(subst .,-,${cuda_version})
cuda_packages := cuda-minimal-build-${cuda_tag} \
                 cuda-nvtx-${cuda_tag} cuda-nvprof-${cuda_tag} \
                 libcublas-${cuda_tag} libcublas-dev-${cuda_tag} \
                 libcusolver-${cuda_tag} libcusolver-dev-${cuda_tag} \
                 libcusparse-${cuda_tag} libcusparse-dev-${cuda_tag} \
                 libnvjitlink-${cuda_tag} libnvjitlink-dev-${cuda_tag}


# N.B. important to install devel package to make the libraries discoverable by the wfn91 linalg cmake kit
intel_mkl := intel-oneapi-mkl-devel

install/core:
	apt-get -yq update
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
	  libmpich-dev mpich libscalapack-mpich-dev \
          python3 python3-pip python3-numpy python3-dev python3-pytest \
          vim emacs-nox ccache

install/main/noble: install/core install/g++ install/clang++ install/g++-14
	apt-get -yq install libboost-all-dev

# VRG stack requires Boost 1.81 or later now, so stock Boost no longer suffices for Ubuntu22.
install/main/jammy: install/core install/g++ install/clang++ install/g++-12
	add-apt-repository -y ppa:mhier/libboost-latest
	apt-get -yq update
	apt-get -yq install libboost1.81-all-dev

install/main: install/main/${ubuntu_codename}

# Default g++
install/g++:
	apt-get -yq install gcc g++ gfortran

install/g++-%:
	apt-get -yq install gcc-$* g++-$* libstdc++-$*-dev gfortran-$*

# Default clang++
install/clang++:
	apt-get -yq install clang libomp-dev

# Specific LLVM version
install/clang++-%:
	wget https://apt.llvm.org/llvm-snapshot.gpg.key -O - | apt-key add -
	add-apt-repository "deb http://apt.llvm.org/${ubuntu_codename}/ llvm-toolchain-${ubuntu_codename}-$* main"
	apt-get -yq update
	apt-get -yq install clang-$* libomp-$*-dev

# CMake
install/cmake:
	wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add -
	apt-add-repository "deb https://apt.kitware.com/ubuntu/ ${ubuntu_codename} main"
	apt-get -yq update
	apt-get -yq install cmake cmake-curses-gui

# Intel
apt-add-repository/intel-mkl:
	wget -O - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
	echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list
	apt-get -yq update

download/intel-mkl:
	apt-get -yq install --download-only ${intel_mkl}

install/intel-mkl:
	apt-get -yq install ${intel_mkl}
	test -x /opt/intel/oneapi/mkl/latest/env/vars.sh

# CUDA
apt-add-repository/cuda:
	wget ${cuda_repo}/cuda-ubuntu$(subst .,,${ubuntu_release}).pin -O /etc/apt/preferences.d/cuda-repository-pin-600
	wget ${cuda_repo}/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb
	sudo dpkg -i /tmp/cuda-keyring.deb
	rm /tmp/cuda-keyring.deb
	#apt-key adv --fetch-keys ${cuda_repo}/7fa2af80.pub
	add-apt-repository "deb ${cuda_repo} /"
	apt-get -yq update

download/cuda:
	apt-get -yq install --download-only ${cuda_packages}

install/cuda: download/cuda
	apt-get -yq install ${cuda_packages}
	rm -fr /usr/local/cuda
	ln -s /usr/local/cuda-$(subst -,.,${cuda_tag}) /usr/local/cuda
	rm -fr /usr/local/bin/nvcc
	# NB /usr/local/cuda/bin/nvcc symlink breaks CMake
	#ln -s /usr/local/cuda/bin/nvcc /usr/local/bin/nvcc

install/cuda-drivers: download/cuda
	apt-get -yq install cuda-drivers

install/nvidia-container-toolkit:
	test -x `which curl`
	curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
	curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
          | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
          | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
	sudo apt-get -yq update
	sudo apt-get -yq install -y nvidia-container-toolkit
	sudo nvidia-ctk runtime configure --runtime=docker
	sudo systemctl restart docker
	# sudo nvidia-ctk runtime configure --runtime=containerd
	# sudo systemctl restart containerd

# Libint2
libint2:
	wget --no-verbose https://github.com/evaleev/libint/releases/download/v${libint2_version}/libint-${libint2_version}-mpqc4.tgz
	tar xzf libint-${libint2_version}-mpqc4.tgz
	mkdir libint2
	cd libint2 && cmake ../libint-${libint2_version} -DCMAKE_BUILD_TYPE=Release -DLIBINT2_BUILD_SHARED_AND_STATIC_LIBS=ON
	cd libint2 && make -j16

.PHONY: install/libint2
install/libint2: libint2
	make -C libint2 install

# Libxc
libxc:
	wget https://gitlab.com/libxc/libxc/-/archive/${libxc_version}/libxc-${libxc_version}.tar.bz2
	tar xf libxc-${libxc_version}.tar.bz2
	cd libxc-${libxc_version} && sed -i -e "s/p->info->family != XC_KINETIC/p->info->kind != XC_KINETIC/g" src/work_mgga_inc.c
	mkdir libxc
	cd libxc && cmake ../libxc-${libxc_version} -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
	cd libxc && make -j16

.PHONY: install/libxc
install/libxc: libxc
	make -C libxc install

install/docker:
	sudo dpkg -P snapd
	sudo apt-get -yq install docker.io
