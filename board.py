import board
import digitalio
import busio
import usb_cdc
import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import storage

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_bluefruit_connect.packet import Packet
# Only the packet classes that are imported will be known to Packet.
from adafruit_bluefruit_connect.button_packet import ButtonPacket

ble = BLERadio()
ble.name = "RobotArm"
uart_server = UARTService()
advertisement = ProvideServicesAdvertisement(uart_server)
print(ble.name)

pixel_pin = board.P0_17
num_pixels = 40
i2c = busio.I2C(board.P0_06, board.P0_08)
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
write_timeout = 0

led.value = 0

pca = PCA9685(i2c)
pca.frequency = 200
armjoint = []
armjoint.append(servo.Servo(pca.channels[0], min_pulse=400, max_pulse=2500 ))
armjoint.append(servo.Servo(pca.channels[1], min_pulse=400, max_pulse=2500 ))
armjoint.append(servo.Servo(pca.channels[2], min_pulse=400, max_pulse=2500 ))
armjoint.append(servo.Servo(pca.channels[3], min_pulse=400, max_pulse=2500 ))
armjoint.append(servo.Servo(pca.channels[4], min_pulse=400, max_pulse=2500 ))
armjoint.append(servo.Servo(pca.channels[5], min_pulse=400, max_pulse=2500 ))

# Define the interval in seconds
INTERVAL = 0.03  # 100 milliseconds
SMALL_ANGLE = 10

Recording = False
Recorded_motion = []

##### WARNNING!!!!  #####
#running the below line will make the USB readonly from Windows, then have to use REPL to execute
# the opposite of storage.remount("/", readonly=True) after EJECT USB drive from windows
#storage.remount("/", readonly=False)

armjoint[0].angle = 90
time.sleep(0.3)
armjoint[1].angle = 90
time.sleep(0.3)
armjoint[2].angle = 90
time.sleep(0.3)
armjoint[3].angle = 90
time.sleep(0.3)
armjoint[4].angle = 90
time.sleep(0.3)
armjoint[5].angle = 40
time.sleep(0.3)
current_pos = [90, 90, 90, 90, 90, 40] #nominal position safe for shutdown
safe_pos = [80, 120, 140, 90, 60, 30]
rest_pos = [80,0,180,180,60,30]
straight_up = [80,100,70,90,150,30]

buffer = ""
serial = usb_cdc.console

def read_serial(serial):
    text_read_ok=0
    available = serial.in_waiting
    while available:
        raw = serial.read(available)
        text = raw.decode("utf-8")
        available = serial.in_waiting
        text_read_ok = 1
    if text_read_ok:
        return text

def read_file_into_list(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    return [line.strip() for line in lines]

# Define the filename
filename = 'Recording.txt'

# Read the file into a list and print the result
#lines = read_file_into_list(filename)
#print(lines)


def fastmove_to(new_pos):
    global current_pos

    for i in range(6):
        armjoint[i].angle = new_pos[i]
    current_pos = new_pos

def fastmove_by(motion):
    global current_pos

    new_pos = [x + y for x, y in zip(current_pos, motion)]
    fastmove_to(new_pos)

def web_move(newpos, speed):
    global current_pos
    newnewpos = [current_pos[0], newpos[0],newpos[1],newpos[2],current_pos[4], current_pos[5]]
    move_to(newnewpos, speed)

def turn(newpos, speed):
    global current_pos
    newnewpos = [newpos, current_pos[1], current_pos[2],current_pos[3],current_pos[4], current_pos[5]]
    move_to(newnewpos, speed)

def twist(newpos,speed):
    global current_pos
    newnewpos = [current_pos[0], current_pos[1], current_pos[2],current_pos[3], newpos, current_pos[5]]
    move_to(newnewpos, speed)

def jaw(newpos,speed):
    global current_pos
    print("attempt")
    newnewpos = [current_pos[0], current_pos[1], current_pos[2],current_pos[3],current_pos[4], newpos]
    move_to(newnewpos, speed)

def move_to(new_pos,speed): #blocking, but can be changed to just return after one move
    global current_pos, Recording, Recorded_motion

    while current_pos != new_pos: #Simultaneous motion for all motors
        for i in range(6):
            if current_pos[i] != new_pos[i]:
                diff_angle = new_pos[i]- current_pos[i]
                if diff_angle>0:
                    #move servo by SMALL_ANGLE
                    if diff_angle>SMALL_ANGLE:
                        current_pos[i] += SMALL_ANGLE
                        armjoint[i].angle = current_pos[i]
                    else:
                        current_pos[i] = new_pos[i]
                        armjoint[i].angle = new_pos[i]
                if diff_angle<0:
                    #move servo by SMALL_ANGLE
                    if diff_angle< (-SMALL_ANGLE):
                        current_pos[i] -= SMALL_ANGLE
                        armjoint[i].angle = current_pos[i]
                    else:
                        current_pos[i] = new_pos[i]
                        armjoint[i].angle = new_pos[i]
                if speed != 0:
                    time.sleep(1/(speed*10))
                else:
                    continue
    print(current_pos)
    if Recording == True:
        Recorded_motion.append([current_pos,speed])
        #print(current_pos)



def move_by(motion,speed):
    global current_pos

    new_pos = [x + y for x, y in zip(current_pos, motion)]
    move_to(new_pos,speed)


def gesture(sequence,speed):
    global current_pos

    for i in range(len(sequence)):
        motion = sequence[i]
        move_by(motion,speed)


#### PRIMITIVE MOTION ####

def arm_ccw(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[0] = new_pos[0] + degree
    if new_pos[0] <= 180:
        move_to(new_pos,speed)

def arm_cw(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[0] = current_pos[0] - degree
    if new_pos[0] >= 0:
        move_to(new_pos,speed)

def arm_back(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[1] = current_pos[1] + degree
    new_pos[2] = current_pos[2] + degree
    if new_pos[1] <= 180 and new_pos[2] <=180:
        move_to(new_pos,speed)

def arm_forward(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[1] = current_pos[1] - degree
    new_pos[2] = current_pos[2] - degree
    if new_pos[1] >= 0 and new_pos[2] >= 0:
        move_to(new_pos,speed)

def elbow_down(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[3] = current_pos[3] + degree
    if new_pos[3] <= 180:
        move_to(new_pos,speed)

def elbow_up(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[3] = current_pos[3] - degree
    if new_pos[3] >= 0:
        move_to(new_pos,speed)

def wrist_cw(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[4] = current_pos[4] - degree
    if new_pos[4] >= 0:
        move_to(new_pos,speed)

def wrist_ccw(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[4] = current_pos[4] + degree
    if new_pos[4] <= 180:
        move_to(new_pos,speed)

def jaw_close(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[5] = current_pos[5] + degree
    if new_pos[5] <= 60:
        move_to(new_pos,speed)

def jaw_open(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[5] = current_pos[5] - degree
    if new_pos[5] >= 0:
        move_to(new_pos,speed)

def swing_up(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[1] = current_pos[1] + degree
    if new_pos[1] <= 180:
        move_to(new_pos,speed)

def swing_down(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[1] = current_pos[1] - degree
    if new_pos[1] >= 0:
        move_to(new_pos,speed)

#### INDIVIDUAL SERVO MOTION ####
def arm_rotate_to(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[0] = degree
    if new_pos[0] >= 0 and new_pos[0] <= 180:
        move_to(new_pos,speed)

def arm_up_to(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[1] = degree
    if new_pos[1] >= 0 and new_pos[1] <= 180:
        move_to(new_pos,speed)

def elbow_rotate_to(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[2] = degree
    if new_pos[2] >= 0 and new_pos[2] <= 180:
        move_to(new_pos,speed)

def wrist_move_to(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[3] = degree
    if new_pos[3] >= 0 and new_pos[3] <= 180:
        move_to(new_pos,speed)

def jaw_rotate_to(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[4] = degree
    if new_pos[4] >= 0 and new_pos[4] <= 180:
        move_to(new_pos,speed)

def jaw_open_to(degree,speed):
    global current_pos
    new_pos = list(current_pos)
    new_pos[5] = degree
    if new_pos[5] >= 0 and new_pos[5] <= 60:
        move_to(new_pos,speed)


BLE_enabled = False

# print("WAITING...")
# #Advertise when not connected.
# ble.start_advertising(advertisement)
# while not ble.connected:
#     pass
# # Connected
# ble.stop_advertising()
# print("CONNECTED")

while 1:
    # Read BLE packets
    if BLE_enabled == True:
        if ble.connected:
            ble.stop_advertising()

            # Keeping trying until a good packet is received
            try:
                packet = Packet.from_stream(uart_server)
            except ValueError:
                continue

            # Only handle button packets
            if isinstance(packet, ButtonPacket) and packet.pressed:
                if packet.button == ButtonPacket.UP:
                    arm_back(3,3)
                    print("Button UP")
                if packet.button == ButtonPacket.DOWN:
                    arm_forward(3,3)
                    print("Button DOWN")
                if packet.button == ButtonPacket.LEFT:
                    arm_cw(3,3)
                    print("Button LEFT")
                if packet.button == ButtonPacket.RIGHT:
                    arm_ccw(3,3)
                    print("Button RIGHT")
                if packet.button == ButtonPacket.BUTTON_1:
                    elbow_up(3,3)
                    print("Button 1")
                if packet.button == ButtonPacket.BUTTON_2:
                    elbow_down(3,3)
                    print("Button 2")
                if packet.button == ButtonPacket.BUTTON_3:
                    jaw_open(5,10)
                    print("Button 3")
                if packet.button == ButtonPacket.BUTTON_4:
                    jaw_close(5,10)
                    print("Button 4")
        # Disconnected
        else:
            try:
                ble.start_advertising(advertisement)
            except Exception:
                continue
            print("BLE disconnected")

    # Read USB serial port
    char_read = read_serial(serial)
    if char_read:
        buffer += char_read
        #print("buffer is:", buffer)
        input_str = buffer.strip().replace(" ", "")
        arg = input_str.split(',')

        if len (arg) == 5:
            #print("starting webarm")
            motion = arg[0]
            if motion == "webArm":
                angle1 = abs(int(arg[1]))
                angle2 = abs(180-int(arg[2]))
                angle3 = abs(180-int(arg[3]))
                speed =  int(arg[4])
                newpos = [angle1, angle2, angle3]
                web_move(newpos, speed)
        elif len(arg) == 100000:
            print(buffer)
            motion = arg[0]
            degree = int(arg[1])
            speed = int(arg[2])
            if motion == "armcw":
                arm_cw(degree,speed)
            elif motion == "armccw":
                arm_ccw(degree,speed)
            elif motion == "armbk":
                arm_back(degree,speed)
            elif motion == "armfw":
                arm_forward(degree,speed)
            elif motion == "elbowup":
                _up(degree,speed)
            elif motion == "elbowdown":
                elbow_down(degree,speed)
            elif motion == "wristcw":
                wrist_cw(degree,speed)
            elif motion == "wristccw":
                wrist_ccw(degree,speed)
            elif motion == "jawopen":
                jaw_open(degree,speed)
            elif motion == "jawclose":
                jaw_close(degree,speed)
            elif motion == "swup":
                swing_up(degree,speed)
            elif motion == "swdown":
                swing_down(degree,speed)
            elif motion == "armrotto":
                arm_rotate_to(degree,speed)
            elif motion == "armupto":
                arm_up_to(degree,speed)
            elif motion == "elbowrotto":
                elbow_rotate_to(degree,speed)
            elif motion == "wristmoveto":
                wrist_move_to(degree,speed)
            elif motion == "jawrotto":
                jaw_rotate_to(degree,speed)
            elif motion == "jawopento":
                jaw_open_to(degree,speed)

        elif len(arg) == 2:
            motion = arg[0]
            speed = int(arg[1])
            if motion == "gostraight":
                move_to(straight_up, speed)
            elif motion == "gosafe":
                move_to(safe_pos, speed)
            elif motion == "turn":
                turn(speed,2) #speed is now just the angle
            elif motion == "jaw":
                jaw(speed,2)
            elif motion == "twist":
                twist(speed,2)

        elif len(arg) == 1:
            cmd = arg[0]
            if cmd == "record":
                Recording = True
                print("Start recording\n")
            elif cmd == "save":
                Recording = False
                print(Recorded_motion)
#                 with open('Recording.txt', 'w') as file:
#                     for sublist in list_of_lists:
#                         file.write(' '.join(map(str, sublist)) + '\n')
#                 print("Saved to recording.txt\n")
            elif cmd == "BLEen":
                BLE_enabled = True
            elif cmd == "BLEdis":
                BLE_enabled = False

        else:
            print("cmd not found")

    if buffer.endswith("\n"):
        # strip line end
        input_line = buffer[:-1]
        # clear buffer
        buffer = ""
        # handle input
        #print(input_line)



#     try:
#         input_str = input("ready").strip().replace(" ", "")
#     except Exception as e:
#         print("input error")
#     arg = input_str.split(',')
#     if len(arg) == 3:
#         motion = arg[0]
#         degree = int(arg[1])
#         speed = int(arg[2])
#         if motion == "armcw":
#             arm_cw(degree,speed)
#         elif motion == "armccw":
#             arm_ccw(degree,speed)
#         elif motion == "armbk":
#             arm_back(degree,speed)
#         elif motion == "armfw":
#             arm_forward(degree,speed)
#         elif motion == "angleup":
#             angle_up(degree,speed)
#         elif motion == "angledown":
#             angle_down(degree,speed)
#         elif motion == "wristcw":
#             wrist_cw(degree,speed)
#         elif motion == "wristccw":
#             wrist_ccw(degree,speed)
#         elif motion == "jawopen":
#             jaw_open(degree,speed)
#         elif motion == "jawclose":
#             jaw_close(degree,speed)
#         elif motion == "swup":
#             swing_up(degree,speed)
#         elif motion == "swdown":
#             swing_down(degree,speed)
#         else:
#             print("cmd not found")




if 0:
#### SHOW 6-DOF
    for i in range(6):
        print(i)
        input()
        motion = [0,0,0,0,0,0]
        motion[i] = -30
        fastmove_by(motion)
        time.sleep(0.5)
        motion[i]=30
        fastmove_by(motion)

    input("move to face me")

    #### SHAKE HAND
    handup= [0,0,0,-30,0,0]
    handdown = [0,0,0,30,0,0]
    shakehand = [handdown, handup, handdown, handup, handdown, handup]
    move_by([-80,0,-30,30,0,0],7)
    input("shake hand")
    move_by([0,0,0,0,0,50],7)
    gesture(shakehand,10)




    #### WAVE
    right = [0,0,40,60,0,0]
    left = [0,0,-40,-60,0,0]
    wave = [right,left,right,left]
    input("straight up")
    move_to(straight_up,10)
    input("wave now?")
    move_by([-50,0,0,0,-90,0],10)
    gesture(wave,10)
    move_by([50,0,0,0,0,0],10)
    gesture(wave,10)
    move_by([50,0,0,0,0,0],10)
    gesture(wave,10)

    time.sleep(2)
    #### THROW SIDEWAY
    move_by([-100,0,70,0,90,0],10)
    #move_by([-50,0,0,0,0,0],0.02)
    input("close jaw")
    move_by([0,0,0,0,0,40],5)
    input("throw")
    fastmove_by([120,0,0,0,0,0])
    time.sleep(0.33)
    fastmove_by([0,0,0,0,0,-30])

    input("lean back")
    move_to(straight_up,10)
    move_by([0,0,-30,0,-90,0],10)
    input("get candy again")
    move_by([0,0,0,0,0,40],5)
    input("throw big")
    fastmove_by([0,0,90,0,0,0])
    time.sleep(0.25)
    fastmove_by([0,0,0,0,0,-40])

input("back to safe")
move_to(safe_pos,10)

repeat_pos = current_pos
print(repeat_pos)
for i in range(100):
    move_by([0,0,0,10,-90,-30],10)
    move_by([0,-70,0,-70,0,0],5)
    time.sleep(1)
    move_by([0,0,-60,60,0,0],3)

    input("pick up cup here")
    move_by([0,0,0,0,0,30],5)
    move_by([0,70,0,70,0,0],3)
    move_by([0,0,-20,20,0,0],3)

    time.sleep(3)
    move_to(safe_pos,10)
    input("press to repeat")


#move up to drink
move_by([-80,0,0,0,0,0],5)
time.sleep(2)
move_by([0,0,0,0,60,0],5)
time.sleep(2)
move_by([0,0,0,0,-60,0],5)
time.sleep(2)
move_by([80,0,0,0,0,0],5)
time.sleep(1)
fastmove_by([-80,0,0,0,80,-30])
time.sleep(2)
for i in range(6):
    fastmove_by([80,0,0,0,0,0])
    time.sleep(0.5)
    fastmove_by([-80,0,0,0,0,0])
    time.sleep(0.5)
    fastmove_by([0,0,-60,0,0,0])
    time.sleep(0.5)
    fastmove_by([0,0,60,0,0,0])
    time.sleep(0.5)

time.sleep(5)
move_to(safe_pos,10)

pca.deinit()




# while True:
#     serial_in = input()
#     if serial_in == "2":
#         led.value = 1
#     else:
#         led.value = 0
#     print(serial_in)
