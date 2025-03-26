#!/bin/bash

xhost +local:root

DOCKER_OPTIONS=(
    -it --rm
    # --gpus all 
    --name="nav2-gz"
    --network=host --ipc=host
    --env="DISPLAY"
    --env="QT_X11_NO_MITSHM=1"
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --device=/dev/dri:/dev/dri
    --env="XDG_RUNTIME_DIR=/tmp/runtime-dir"
)


SCRIPT_ARGS=()
CMD_ARGS=() # Arguments to pass to the command (everything after --)

# Use a flag to detect when we've hit --
FOUND_SEPARATOR=false

for arg in "$@"; do
    if [ "$arg" == "--" ]; then
        FOUND_SEPARATOR=true
        continue
    fi
    if [ "$FOUND_SEPARATOR" == false ]; then
        SCRIPT_ARGS+=("$arg")
    else
        CMD_ARGS+=("$arg")
    fi
done
    

# Instead of mounting the whole workspace, mount only the frequently 
# changed files and folders. This is necessary because by default, 
# files generated in the container (ie. build/ install/ log/) are owned 
# by root which makes it cumbersome to modify/delete them on the host system.

# Mount src/ directory as read-only
DOCKER_OPTIONS+=(
    --mount type=bind,source="$(pwd)/src",destination=/ws/src,readonly \
)

# Mount all shell scripts and python files in the root of the project directory as read-only
for file in *.sh *.py *.bash .artefactsignore *.yaml *.repos; do
    if [[ -f $file ]]; then
        DOCKER_OPTIONS+=(
            --mount type=bind,source="$(pwd)/$file",destination=/ws/"$file",readonly
        )
    fi
done

# If no CMD_ARGS were provided, default to bash
if [ ${#CMD_ARGS[@]} -eq 0 ]; then
    CMD_ARGS=("/bin/bash")
fi

docker run \
    "${DOCKER_OPTIONS[@]}" \
    "${ISAAC_OPTIONS[@]}" \
    nav2-gz \
    "${CMD_ARGS[@]}"

xhost -local:root