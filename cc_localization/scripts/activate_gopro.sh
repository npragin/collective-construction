#!/usr/bin/env bash
set -e

sudo gopro webcam || {
    echo "gopro webcam failed. Have you installed https://github.com/jschmid1/gopro_as_webcam_on_linux ?"
    exit 1
}

ffmpeg -nostdin -threads 1 \
  -i 'udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=50000000' \
  -f:v mpegts -fflags nobuffer \
  -vf format=yuv420p \
  -f v4l2 /dev/video42