# bridgeモードじゃなくてhostモードでも大丈夫
docker0_ip=172.17.0.1
force_build=true
ROBOT_NAME_VS060=VS060A3-AV6-W4N-ANN

init:
	sed -i s/VS060A3-AV6-NNN-NNN/${ROBOT_NAME_VS060}/ ./src/melodic/denso_robot_ros/denso_robot_descriptions/vs060_description/vs060.launch.xml
	docker network create ros_dev_external
	docker network create \
  --driver=bridge \
  --subnet 192.168.11.0/24 \
  --gateway 192.168.11.50 \
  --opt "com.docker.network.bridge.name"="br0" \
  ros_dev_external_with_hand

ifeq ($(force_build), true)
	docker build -f docker/melodic/Dockerfile.base -t yamachaso/ros_melodic_base:latest .
	docker build -f docker/noetic/Dockerfile.base -t yamachaso/ros_noetic_base:latest .
else
	docker pull yamachaso/ros_melodic_base:latest
	docker pull yamachaso/ros_noetic_base:latest
endif

start:
	xhost + localhost
	ROS_MASTER_DOMAIN=ros_melodic docker compose up -d --force-recreate

start-hand:
	xhost + localhost
	ROS_MASTER_DOMAIN=ros_melodic docker compose -f docker-compose-with-hand.yml up -d --force-recreate

# localhostなどでは失敗する | ref: https://www.finnrietz.dev/linux/ros-docker/
# start-host:
# 	xhost + localhost
# 	ROS_MASTER_DOMAIN=${docker0_ip} docker compose up -d --force-recreate
# 	roscore

stop:
	xhost - localhost
	docker compose stop

melodic:
	docker compose exec ros_melodic /bin/bash

noetic:
	docker compose exec ros_noetic /bin/bash
