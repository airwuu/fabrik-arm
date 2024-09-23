# Fabrik Arm
An implementation of Forward and Backwards Reaching Inverse Kinematics (FABRIK) in Python to calculate relative angles for each motor in a custom robot arm. 
I also used matplotlib to interface with my FABRIK implementation, as demonstrated below.

## Demo
![IMG_5788-ezgif com-optimize (1) (1)](https://github.com/user-attachments/assets/7ceab1d9-8b0d-48e4-9cd4-9240892c825e)

## Usage
For those of you who built an robot arm with 3 segments on one plane, feel free to modify the segment lengths and try:
1. clone the repo and run mat.py
```sh
git clone https://github.com/airwuu/fabrik-arm
cd fabrik-arm
```
2. edit the segment variables to match the proportions on your robot arm!
3. move board.py onto the `SuperMini nRF52840 Pro Micro BLE Controller` and run it on the board
```sh
python board.py
```
4. make sure mat.py has the right `/dev/[your serial port]` and run it
```sh
python mat.py
``` 



