CORES_PER_JOB ?= 12
GITLAB_CONCURRENT ?= 4

gitlab-runner-register = gitlab-runner register \
  --url https://gitlab.com

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
  --env CMAKE_BUILD_PARALLEL_LEVEL=${CORES_PER_JOB} \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --executor docker \
  --docker-runtime nvidia \
  --docker-cpus "${CORES_PER_JOB}" \
  --docker-gpus all \
  --docker-pull-policy always \
  --docker-volumes "/cache" \
  --docker-volumes "/root/.ccache:/root/.ccache:rw" \
  --docker-image valeevgroup/ubuntu
	echo "Make sure docker.io is installed: make install/docker"
	echo "Make sure nvidia-docker is installed: make install/nvidia-container-runtime"

gitlab-runner/unregister/all:
	sudo gitlab-runner unregister --all-runners
