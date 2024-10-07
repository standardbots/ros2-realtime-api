# Runs interactive shell in docker container

# run ./run_shell.sh
# then cd /src

docker run --rm --net=host -it -v .:/src --privileged /bin/bash
