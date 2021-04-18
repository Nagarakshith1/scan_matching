# Scan Matching
![](https://github.com/Nagarakshith1/scan_matching/blob/main/gif/scan_matching.gif)

## Dependencies
[f1tenth](https://github.com/f1tenth/f1tenth_simulator)

## Build with Dockerfile
```
$ docker build . -t scan_matching:0.1
```
## Running the docker image
```
$ xhost +
```
```
$ docker run -it --rm --name test -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro scan_matching:0.1
```
