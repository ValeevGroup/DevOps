# Libint2: the library (not the compiler), from an exported Libint library tarball.
# Kept separate from Makefile: the Dockerfile copies this file right before
# installing Libint in the last layer, so that changing the Libint version
# (or tarball) rebuilds only that layer.
libint2_version ?= 2.14.1
libint2_tarball ?= libint-${libint2_version}-mpqc4.tar.xz
libint2_url ?= https://github.com/evaleev/libint/releases/download/v${libint2_version}/${libint2_tarball}
libint2_srcdir = $(basename $(basename ${libint2_tarball}))
nproc ?= $(shell nproc)

libint2:
	wget --no-verbose ${libint2_url} -O ${libint2_tarball}
	tar xf ${libint2_tarball}
	mkdir libint2
	cd libint2 && cmake ../${libint2_srcdir} -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
	cd libint2 && make -j${nproc}

.PHONY: install/libint2
install/libint2: libint2
	make -C libint2 install

.PHONY: clean/libint2
clean/libint2:
	rm -fr libint2 ${libint2_srcdir} ${libint2_tarball}
