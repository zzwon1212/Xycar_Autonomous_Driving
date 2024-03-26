
# 0. Installation
## 0.1. In local
```
git clone https://github.com/zzwon1212/Xycar_Autonomous_Driving.git xycar_ws/src

docker build --no-cache \
             --progress=tty \
             --force-rm \
             --file xycar_ws/src/ros-opencv.dockerfile \
             --tag ros-opencv:base .

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

## 0.2. In container
```
mkdir /workspace/xycar_ws/src/model
gdown https://drive.google.com/uc?id=15BVH0khkhQEJEHxrZmIOASWVtVg40j_e -O - --quiet | \
tar -xz -C /workspace/xycar_ws/src/model

mkdir /workspace/xycar_ws/src/video
gdown https://drive.google.com/uc?id=1Pf1bpd4mr-lMeqC3e6McHlktn2gJpRET -O - --quiet | \
tar -xz -C /workspace/xycar_ws/src/video
```
You need model weights to predict objects and raw image rosbag from usb camera to watch the result. (optional: video from smartphone to concatenate with the result)

Download these by using `gdown` code above or using the given link below yourself.

- model weights [link](https://drive.google.com/file/d/15BVH0khkhQEJEHxrZmIOASWVtVg40j_e/view?usp=drive_link)

- raw image rosbag and video from smartphone [link](https://drive.google.com/file/d/1Pf1bpd4mr-lMeqC3e6McHlktn2gJpRET/view?usp=drive_link)

```
cd /workspace/xycar_ws/src/yolov3_onnx_rt
python3 onnx_to_tensorrt.py --cfg yolov3-tiny_tstl_416.cfg \
                            --onnx ../model/model_epoch4400_pretrained.onnx \
                            --num_class 8 \
                            --input_img sample.png
```

```
cd /workspace/xycar_ws
catkin_make || true && catkin_make
source devel/setup.bash
```
Build your catkin wrokspace.


git clone -b release/8.2 https://github.com/nvidia/TensorRT TensorRT
cd TensorRT
git submodule update --init --recursive


# 0. Run
```
roslaunch autonomous_driving drive.launch
```
Run main node using this code.

If you want to watch opencv window, run `xhost +` at your local CMD.

If you modify C++ files, run `catkin_make` at `/xycar_ws` directory to build again.


ALL OPTIONAL BELOW.

# 0. YOLO
## 0.1. Data
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

## 0.2. Train
at `yolov3_pytorch` directory
```
python main.py --mode train --cfg ./cfg/yolov3-tiny_tstl_416.cfg --gpus 0 (--checkpoint ./output/YOUR.pth) (--pretrained yolov3-tiny.weights)
```
I used pretrained weights (YOLOv3-tiny Darknet Weights [link](https://pjreddie.com/media/files/yolov3-tiny.weights)).

You can get `.pth` weights by running the code above.

### Tensorboard
```
tensorboard --logdir=./output --port 8888
```
You can monitor the tensorboard at http://localhost:8888/

```
ssh -N -L 8888:localhost:8888 ubuntu@IPv4
```
If you use server like AWS, run above code at your local cmd.

## 0.3. Convert `.pth` to `.weights`
```
python main.py --mode onnx --cfg ./cfg/yolov3-tiny_tstl_416.cfg --gpus 0 --checkpoint ./output/model_epoch4400_pretrained.pth
```
If you want to convert `.pth` to `.weights`, change `ONNX_EXPORT = True` at line #10 in `model/yolov3.py` and run the code above.


## 0.4 Convert `.weights` to `.onnx`
at `yolov3_onnx_rt` directory

```
python yolov3_to_onnx.py --cfg ../yolov3-pytorch/cfg/yolov3-tiny_tstl_416.cfg --weights ../yolov3-pytorch/output/model_epoch4400_pretrained.weights --num_class 8
```
If you want to convert `.weights` to `.onnx`, run the code above.

## 0.5. Optimize `.onnx` to `.trt`
```
vim ~/.bashrc
export OPENBLAS_CORETYPE=ARMV8
python onnx_to_tensorrt.py --cfg ../yolov3-pytorch/cfg/yolov3-tiny_tstl_416.cfg --onnx model_epoch4400.onnx --num_class 8 --input_img 2024-01-18_14-51_frame2240.png
```
If you want to optimize `.onnx` to `.trt`, run the code above at your target device such as NVIDIA Jetson Nano.

## 0.6. Model Weights
```
└─ models
    ├─ model_epoch4400_pretrianed.pth         <-- PyTorch
    ├─ model_epoch4400_pretrianed.weights     <-- Darknet
    ├─ model_epoch4400_pretrianed.onnx        <-- ONNX
    └─ model_epoch4400_pretrianed_04_001.trt  <-- TensorRT
```
All weights are available at section '0.2.' [above](#0.2.-in-container).



# 0. Depth Estimation
