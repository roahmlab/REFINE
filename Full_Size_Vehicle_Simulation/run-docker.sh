cp /etc/passwd $(pwd)/.etc_passwd
getent passwd $(whoami) >> $(pwd)/.etc_passwd

cp /etc/group $(pwd)/.etc_group
getent group $(whoami) >> $(pwd)/.etc_group

xhost +
docker run -it --rm                     \
  --gpus all                            \
  -e DISPLAY=$DISPLAY                   \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro   \
  --shm-size=512M                       \
  --network="host"                      \
  -u $(id -u):$(id -g)                  \
  -ti                                   \
  -v $(pwd)/.etc_passwd:/etc/passwd:ro  \
  -v $(pwd)/.etc_group:/etc/group:ro    \
  -v $(pwd)/docker-home:/home/$(id -nu) \
  -v /dev/shm:/dev/shm                  \
  -v /etc/timezone:/etc/timezone:ro     \
  -v /etc/localtime:/etc/localtime:ro   \
  -v $(pwd)/data:/data:ro               \
  -v $(pwd)/simulator:/simulator        \
  -w /simulator                         \
  roahm/refine_sim /usr/bin/bash
