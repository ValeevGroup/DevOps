# What
This repo contains scripts for building VG containers used for TA and MPQC CI. These containers can also be used locally for development.

# How
- Authenticate with Docker Hub: `docker login -u <docker.com username> --password-stdin`
- Build and push: `cd docker-images && make push/latest`
