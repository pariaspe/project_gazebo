# Project Gazebo

Please refer to https://aerostack2.github.io/_02_examples/gazebo/project_gazebo/index.html for more information.

## Setup using Docker

Go to the root folder of the repository and run:

```bash
xhost + # this will enable gazebo visualization
docker compose up -d # use the -d for keep the container alive in background
```

With this there is a running instance of the container with this project mounted in ```/root/lab_gz```.
Now you can run as much terminals as you need by running: 

```bash
docker exec -it aerostack2_lab /bin/bash
```

> For stopping the container run ```xhost - ; docker compose down ``` command on the repo root folder. This will also remove the access to the XServer from the container.
