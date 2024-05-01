# pioneer
Pioneer Robot Repository for UWA Mobile Robots.

To run the docker image
''' sh
docker image build -t pioneer .
docker run -it --user ros --network=host --ipc=host -v $PWD/docker_shared:/docker_shared --env=DISPLAY pioneer
'''