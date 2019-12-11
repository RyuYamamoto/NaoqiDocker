xhost +si:localuser:root

NVIDIA_VERSION=396

docker run --runtime=nvidia -it --name naoqi_sandbox --rm \
       -v $(pwd):/mnt --workdir=/mnt \
	   -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	   -v /usr/lib/nvidia-${NVIDIA_VERSION}:/usr/lib/nvidia-${NVIDIA_VERSION} \
	   -v /usr/lib32/nvidia-${NVIDIA_VERSION}:/usr/lib32/${NVIDIA_VERSION} \
       --net host \
       --privileged     \
	   -e DISPLAY=$DISPLAY \
       --env="LANG=ja_JP.UTF-8" \
	   --env QT_X11_NO_MITSHM=1 \
       naoqi /bin/bash
