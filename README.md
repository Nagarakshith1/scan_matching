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
Disable nvidia GPU if present and then run the following commands
```
$ xhost +
```
```
$ docker run -it --rm --name scan_matching -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro msnaga/scan_matching:0.1
```
## Simulator controls
1) Press 'k' to activate the keyboard on the running terminal
2) 'w' for forward, 's' for reverse, 'a' for left steer, 'd' for right steer, spacebar for brakes

## Reference
A. Censi, "An ICP variant using a point-to-line metric," 2008 IEEE International Conference on Robotics and Automation, Pasadena, CA, USA, 2008, pp. 19-25, doi: 10.1109/ROBOT.2008.4543181.
