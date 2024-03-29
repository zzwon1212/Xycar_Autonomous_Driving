[<img src="https://github.com/zzwon1212/Xycar_Autonomous_Driving/assets/61040406/3d0200eb-4bf0-486b-8ba9-e2b7da08f06a" width="80%" height="auto">](https://www.youtube.com/watch?v=0QzQx7FWpcM)

# 1. Introduction

# 2. Installation and Preparation
## 2.1. Run Docker Container
```
git clone https://github.com/zzwon1212/Xycar_Autonomous_Driving.git xycar_ws/src
```
Clone this repository at your workspace.

```
docker build --no-cache \
             --progress=tty \
             --force-rm \
             --file xycar_ws/src/ros-opencv.dockerfile \
             --tag ros-opencv:base \
             .
```
Build the Docker image. This may take quite a long time depending on your computing resources.

```
docker run -it \
           --gpus all \
           --volume "$(pwd)":/workspace \
           --volume /tmp/.X11-unix:/tmp/.X11-unix \
           --name "ROS_OpenCV" \
           --shm-size 14G \
           --publish 8888:8888 \
           --env DISPLAY=$DISPLAY \
           ros-opencv:base \
           /bin/bash
```
Run a container using the built image.



## 2.2. Donwload rosbag
```
mkdir /workspace/xycar_ws/src/video
gdown https://drive.google.com/uc?id=1Pf1bpd4mr-lMeqC3e6McHlktn2gJpRET -O - --quiet | \
tar -xz -C /workspace/xycar_ws/src/video
```
Download rosbag file which contains raw images recorded by the car's camera.

Use `gdown` above or by yourself at this [link](https://drive.google.com/file/d/1Pf1bpd4mr-lMeqC3e6McHlktn2gJpRET/view?usp=drive_link).



## 2.3. Prepare model weights
```
mkdir /workspace/xycar_ws/src/model
gdown https://drive.google.com/uc?id=1I5uhP0UB3CNB4ZMh9mtkG_536CkTGGE8 -O - --quiet | \
tar -xz -C /workspace/xycar_ws/src/model
```
Download model weights using `gdown` above or by yourself at this [link](https://drive.google.com/file/d/1I5uhP0UB3CNB4ZMh9mtkG_536CkTGGE8/view?usp=drive_link).

```
└─ models
    ├─ model_epoch4400_pretrianed.pth         <-- PyTorch
    ├─ model_epoch4400_pretrianed.weights     <-- Darknet
    └─ model_epoch4400_pretrianed.onnx        <-- ONNX
```
This file contains various type of model weights. You can see above.

```
cd /workspace/xycar_ws/src/yolov3_onnx_rt
python3 onnx_to_tensorrt.py --cfg yolov3-tiny_tstl_416.cfg \
                            --onnx ../model/model_epoch4400_pretrained.onnx \
                            --num_class 8 \
                            --input_img sample.png
```
Convert `.onnx` format to `.trt` format to **optimize the weights** for your target device (in my case, NVIDIA Jetson Nano).

This process is necessary to get high fps. In my case, `.onnx` get 6 fps, but `.trt` get 60~70 fps.



## 2.4. Build this project
```
cd /workspace/xycar_ws
catkin_make
catkin_make
source devel/setup.bash
```
Build your catkin wrokspace.

With this step, all preparations are now complete.



# 3. Run
```
roslaunch autonomous_driving drive.launch
```
Run main ROS node.

If you want to watch opencv window, run `xhost +` at your local CMD.

If you modify C++ files, run `catkin_make` at `/xycar_ws` directory to build again.



***
# 0. OPTIONAL
## 0.1. Concatenate videos
```
cd /workspace/xycar_ws/src
python3 concatenate.py --input1 video/phone.mp4 \
                       --input2 video/xycar.mp4 \
                       --output video/concatenated.mp4
```
Concatenate the result with a video recorded by smartphone.



## 0.2. YOLO
### 0.2.1. Data
```
└─ yolov3_pytorch
    └─ datasets
        ├─ train
        |   ├─ Annotations
        |   |   └─ *.txt
        |   ├─ ImageSets
        |   |   └─ all.txt
        |   └─ JPEGImages
        |       ├─ *.jpg
        |       └─ *.png
        └─ valid
            ├─ Annotations
            ├─ ImageSets
            └─ JPEGImages
```
You can download datasets used to train my YOLO v3 at this [link](https://drive.google.com/file/d/1uTCYlCCoMkP96kBGcJTR4m5HPh3DCBBc/view?usp=drive_link).



### 0.2.2. Train
```
cd /workspace/xycar_ws/src/yolov3_pytorch
python main.py --mode train --cfg ./cfg/yolov3-tiny_tstl_416.cfg --gpus 0 (--checkpoint ./output/YOUR.pth) (--pretrained yolov3-tiny.weights)
```
I used pretrained weights (YOLOv3-tiny Darknet Weights [link](https://pjreddie.com/media/files/yolov3-tiny.weights)).

You can get `.pth` weights by running the code above.

#### Tensorboard
```
tensorboard --logdir=./output --port 8888
```
You can monitor the tensorboard at http://localhost:8888/

```
ssh -N -L 8888:localhost:8888 ubuntu@IPv4
```
If you use server like AWS, run above code at your local CMD.



### 0.2.3. Convert `.pth` to `.weights`
```
python main.py --mode onnx --cfg ./cfg/yolov3-tiny_tstl_416.cfg --gpus 0 --checkpoint ./output/model_epoch4400_pretrained.pth
```
If you want to convert `.pth` to `.weights`, change `ONNX_EXPORT = True` at line #10 in `model/yolov3.py` and run the code above.


### 0.2.4 Convert `.weights` to `.onnx`
```
cd /workspace/xycar_ws/src/yolov3_onxx_rt
python yolov3_to_onnx.py --cfg ../yolov3-pytorch/cfg/yolov3-tiny_tstl_416.cfg --weights ../yolov3-pytorch/output/model_epoch4400_pretrained.weights --num_class 8
```
If you want to convert `.weights` to `.onnx`, run the code above.



## 0.3. Depth Estimation
