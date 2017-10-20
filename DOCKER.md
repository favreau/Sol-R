# Sol-R with Docker

> Use [Docker](https://docs.docker.com) to run Sol-R as a service and avoid painful tooling setup.

**NOTE**: This is work in progress.

### Prerequisites
-----------------
Head over to [Docker](https://docs.docker.com/engine/installation/#supported-platforms) and install Docker for your own platform.


### Setup
---------
First build the image (*it's necessary to do this step if you want to run Sol-R*):
```bash
docker build . -t Sol-R
```


### Usage
---------
By default, the entrypoint when running the image is `Sol-R`, but if you want to ssh into the container use:
```bash
docker run -ti --rm --entrypoint bash Sol-R
```

If you want to run Sol-R:
```bash
docker run -ti --rm Sol-R
```

**NOTE** If you are having trouble exiting the process after you run the container (with the above command), use `docker stop <container-id>` to stop the container.
`docker ps` will give you the current running process.
