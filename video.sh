#!/bin/bash

source ~/.bashrc

REMOTE_IP=$1

gst-launch-1.0 \
  nvarguscamerasrc sensor-mode=4 ! \
  "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1" ! \
  nvvidconv flip-method=2 ! \
  "video/x-raw(memory:NVMM), format=NV12" ! \
  queue ! \
  nvv4l2h264enc bitrate=2000000 insert-sps-pps=true iframeinterval=60 ! \
  queue ! \
  rtph264pay config-interval=1 pt=96 ! \
  queue ! \
  udpsink host="$REMOTE_IP" port=5600
