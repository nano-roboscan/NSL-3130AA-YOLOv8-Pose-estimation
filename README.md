# NSL-3130AA-YOLOv8-Pose-estimation(OpenCV)
YOLOv8-Pose-estimation + object detection + people counting

## USB rules
```
$ sudo vi /etc/udev/rules.d/defined_lidar.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyLidar"

$ service udev reload
$ service udev restart
```

## Download YOLOv4-CSP-weights
```
$ cd NSL-3130AA-YOLOv8-Pose-estimation/data
$ wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-csp.weights
```

## Required OpenCV
```
- Required OpenCV 4.7
```


## LINUX compile
```
$ cd NSL-3130AA-YOLOv8-Pose-estimation/
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./yolov8 or ./yolov8 -modelType 2 ipaddr xxx.xxx.xxx.xxx or ./yolov8 ipaddr /dev/ttyLidar or ./yolov8 -help
```

![draw](https://github.com/nano-roboscan/NSL-3130AA-YOLOv8-Pose-estimation/assets/106071093/aa0c9a32-9da6-4003-a694-5d73ecd1f474)
