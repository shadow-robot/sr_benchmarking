# Installing the software

In order to use the Shadow Moveit! Planner Benchmarking, you can use our docker image. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. Follow the instructions in the next section to get the latest Docker container up and running.

## Hardware specifications

In order to run our software and the ROS software stack you will need to meet some hardware requirements.

* CPU: Intel i5 or above
* RAM: 4GB or above
* Hard Drive: Fast HDD or SSD (Laptop HDD are very slow)
* Graphics Card: Nvidia GPU (optional)
* OS: Ubuntu 18.04, 16.04 Kinetic (Active development)

## Create docker container

Pull docker image:
```bash
docker pull shadowrobot/dexterous-hand:kinetic-sr-benchmarking
```
Create container:
```bash
docker run -it --privileged --name sr_planner_benchmarking --network=host -e DISPLAY -e QT_X11_NO_MITSHM=1 -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw shadowrobot/dexterous-hand:kinetic-sr-benchmarking
```

```eval_rst
.. Note:: You don’t need to run “docker run” every time as the container is persistent.
```



To start the container again please execute
```
docker start sr_planner_benchmarking
```
