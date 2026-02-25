#!/bin/bash
sleep 5
cd /home/pi/Automotive_v1.2/Instrument_Cluster

# Dùng python hệ thống (đã có đủ PyQt5 + OpenCV vừa cài xong)
/usr/bin/python3 Dashboard_main.py &

sleep 5
/usr/bin/python3 final.py &

exit 0
