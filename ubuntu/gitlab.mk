CMAKE_BUILD_PARALLEL_LEVEL ?= 1
GITLAB_CONCURRENT ?= 1

gitlab-runner-register = gitlab-runner register \
  --url https://gitlab.com \
  --locked=false

gitlab-runner/purge:
	sudo apt purge gitlab-runner
	sudo rm -fr /etc/gitlab-runner/config.toml
	sudo deluser --remove-home gitlab-runner

gitlab-runner/install: /usr/bin/curl
	sudo bash -o pipefail -c \
          'curl -L https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh | bash'
	sudo apt-get update
	export GITLAB_RUNNER_DISABLE_SKEL=true; sudo -E apt-get install gitlab-runner
	sudo usermod -a -G docker gitlab-runner
	echo "Setting 'concurrent = ${GITLAB_CONCURRENT}' in /etc/gitlab-runner/config.toml"
	sudo sed 's/^concurrent = .*$\/concurrent = ${GITLAB_CONCURRENT}/' -i /etc/gitlab-runner/config.toml

gitlab-runner/register/shell:
	sudo ${gitlab-runner-register} \
   --executor shell \
   --tag-list shell,valeevgroup,cuda,linux

gitlab-runner/register/docker:
	sudo ${gitlab-runner-register} \
  --env CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL} \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --docker-runtime nvidia \
  --docker-gpus all \
  --executor docker \
  --tag-list docker,valeevgroup,cuda,linux \
  --docker-pull-policy always \
  --docker-image valeevgroup/ubuntu
	echo "Make sure docker.io is installed: make install/docker"
	echo "Make sure nvidia-docker is installed: make install/nvidia-docker"

gitlab-runner/unregister/all:
	sudo gitlab-runner unregister --all-runners
