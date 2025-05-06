
To build a new Docker image from [`Dockerfile`](Dockerfile), you can run [`docker/build.bash`](docker/build.bash) as shown below.

```bash
docker/build.bash
```

#### Run Container

To run the Docker container, you can use [`docker/run.bash`](run.bash) as shown below.  
The src/ folder and the scripts in the project root are mounted as read-only volumes, so it's not necessary to rebuild the container after changing the source code.

```bash
docker/run.bash
```

#### Join Container

To join a running Docker container from another terminal, you can use [`docker/join.bash`](join.bash) as shown below.

```bash
docker/join.bash
```

#### VS Code Dev Container

As an alternative, VS Code users familiar with [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers) can modify and use the included [`.devcontainer/`](../.devcontainer/devcontainer.json) setup. 

In short, press `Ctrl+Shft+P` in VS Code, and search for the `Reopen in Container` command.
