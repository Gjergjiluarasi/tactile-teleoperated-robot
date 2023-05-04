#! /bin/sh
echo "Running xhost +local:docker to add display capabilities..."
xhost +"local:docker@" # xhost +local:docker
echo "Running docker compose up -d > log/build_docker_compose.log..."
docker compose up -d > log/build_docker_compose.log