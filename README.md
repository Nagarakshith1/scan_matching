# Scan Matching
![](https://github.com/Nagarakshith1/scan_matching/blob/main/gif/scan_matching.gif)

## Dependencies
[f1tenth](https://github.com/f1tenth/f1tenth_simulator)

## Build
### Using Dockerfile
```
$ docker build . -t msnaga/scan_matching:0.1
```
### From dockerhub
```
$ docker pull msnaga/scan_matching:0.1
```
## Running the docker image
```
$ xhost +
```
```
$ docker run -it --rm --name scan_matching -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro msnaga/scan_matching:0.1
```
## Simulator controls
1) Press 'k' to activate the keyboard on the running terminal
2) 'w' for forward, 's' for reverse, 'a' for left steer, 'd' for right steer, spacebar for brakes
