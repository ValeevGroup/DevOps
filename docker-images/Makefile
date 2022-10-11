repo = valeevgroup
latest ?= 20.04
ALL = ${repo}/ubuntu\:18.04-base ${repo}/ubuntu\:18.04-cuda ${repo}/ubuntu\:20.04-base ${repo}/ubuntu\:20.04-cuda
cuda_version = 11.6.2

${repo}/ubuntu\:%-base: build = docker build --build-arg ubuntuImage=ubuntu:$* -f ../ubuntu/Dockerfile ../ubuntu/

${repo}/ubuntu\:%-cuda: build = docker build --build-arg ubuntuImage=nvidia/cuda:${cuda_version}-base-ubuntu$* -f ../ubuntu/Dockerfile ../ubuntu/

${repo}/ubuntu-%: ${repo}/ubuntu\:${latest}-%
	docker tag ${repo}/ubuntu:${latest}-% $@

${repo}/ubuntu\:%:
	${build} -t $@

${repo}/ubuntu\:%.tar:
	mkdir -p ${repo}
	DOCKER_BUILDKIT=1 ${build} -o - > $@

all: ${ALL} ${repo}/ubuntu-base ${repo}/ubuntu-cuda

all/tar: $(ALL:%=%.tar)

push/latest: ${repo}/ubuntu-base ${repo}/ubuntu-cuda
	docker push $?
