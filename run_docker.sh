#!/usr/bin/env bash
#
# Build (if needed) and run the Hybrid A* container with rviz forwarded to the
# host X server. Usage:
#   ./run_docker.sh            # build image if missing, then roslaunch + rviz
#   ./run_docker.sh build      # force a rebuild of the image
#   ./run_docker.sh bash       # drop into an interactive shell in the container
#
set -e

IMAGE=hybrid_a_star:noetic
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

build_image() {
    echo ">>> Building ${IMAGE} ..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
}

case "${1:-run}" in
    build)
        build_image
        exit 0
        ;;
esac

# Build the image on first run.
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    build_image
fi

# Allow the container to talk to the host X server (for rviz).
xhost +local:docker >/dev/null 2>&1 || echo "warning: 'xhost' not available; rviz GUI may not display"

COMMON_ARGS=(
    --rm -it
    --net=host
    --env="DISPLAY=${DISPLAY}"
    --env="QT_X11_NO_MITSHM=1"
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
    # Mount the repo over the baked-in source so edits to maps / launch / rviz /
    # params on the host take effect without rebuilding the image. (Changing C++
    # code still needs a rebuild: ./run_docker.sh build)
    --volume="${SCRIPT_DIR}:/root/hybrid_a_star_ws/src/hybrid_a_star:rw"
)

if [ "${1:-run}" == "bash" ]; then
    docker run "${COMMON_ARGS[@]}" "${IMAGE}" bash
else
    docker run "${COMMON_ARGS[@]}" "${IMAGE}"
fi
