import matplotlib.pyplot as plt
import numpy as np
import csv
from matplotlib.widgets import Button, Slider
from matplotlib.animation import FuncAnimation
from fabrik import *
import serial
import serial.tools.list_ports

serialport = serial.Serial('/dev/ttyACM0', 9600, write_timeout=0)
serialport.timeout = 1

arm = Chain()

class DraggablePoint:
    def __init__(self, point):
        self.point = point
        self.press = None
        self.background = None

    def connect(self):
        self.cidpress = self.point.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.cidrelease = self.point.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.cidmotion = self.point.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        if event.inaxes != self.point.axes: return
        contains, attrd = self.point.contains(event)
        if not contains: return
        self.press = self.point.get_xdata(), self.point.get_ydata(), event.xdata, event.ydata

    def on_release(self, event):
        self.press = None
        self.point.figure.canvas.draw()

    def on_motion(self, event):
        if self.press is None: return
        if event.inaxes != self.point.axes: return
        x0, y0, xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        self.point.set_xdata(x0 + dx)
        self.point.set_ydata(y0 + dy)
        self.point.figure.canvas.draw()
        # print(f'Updated coordinates: ({self.point.get_xdata()[0]}, {self.point.get_ydata()[0]})')
        newpos = arm.fabrik(Vector(int(self.point.get_xdata()[0]), int(self.point.get_ydata()[0])))
        if newpos is not None:
            serialport.write(str(f"webArm, {newpos[0]}, {newpos[1]}, {newpos[2]}, 2\n").encode())

    def reset(self):
        self.point.set_xdata([3])
        self.point.set_ydata([490])
        self.point.figure.canvas.draw()
        newpos = arm.fabrik(Vector(int(self.point.get_xdata()[0]), int(self.point.get_ydata()[0])))
        if newpos is not None:
            serialport.write(str(f"webArm, {newpos[0]}, {newpos[1]}, {newpos[2]}, 2\n").encode())

    def disconnect(self):
        self.point.figure.canvas.mpl_disconnect(self.cidpress)
        self.point.figure.canvas.mpl_disconnect(self.cidrelease)
        self.point.figure.canvas.mpl_disconnect(self.cidmotion)


def extract_coords(filename='coords.csv'):
    """
    Extracts coordinates from a CSV file and returns two arrays: one for x coordinates and one for y coordinates.
    """
    x_coords = []
    y_coords = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header
        for row in reader:
            x_coords.append(float(row[0]))
            y_coords.append(float(row[1]))
    return x_coords, y_coords


#matplotlib madness begins here

init_jaw = 60 #jaw
init_turn = 90 #turn
init_twist = 90

x_coords = [100,200,300,400]
y_coords = [100,200,300,400]

fig, ax = plt.subplots()
plt.get_current_fig_manager().set_window_title('Airs Robot Arm Control')
ax.set_xlim(-200, 400)
ax.set_ylim(-300, 500)

# areate and plot draggable goal/point
point, = ax.plot([30], [490], 'ro', picker=5)
draggable_point = DraggablePoint(point)
draggable_point.connect()
ax.set_xlabel('X')
ax.set_ylabel('Y')
line, = ax.plot(x_coords, y_coords, marker='o', lw=2)

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.25, bottom=0.25)
axfreq = fig.add_axes([0.25, 0.1, 0.65, 0.03])
turn_slider = Slider(
    ax=axfreq,
    label='Turn',
    valmin=0,
    valmax=180,
    valinit=init_turn
)
# Make a vertically oriented slider to control the amplitude
axamp = fig.add_axes([0.12, 0.25, 0.0225, 0.63])
jaw_slider = Slider(
    ax=axamp,
    label="Jaw",
    valmin=30,
    valmax=90,
    valinit=init_jaw,
    orientation="vertical"
)
#another one for twisting hand
axtwist = fig.add_axes([0.04, 0.25, 0.0225, 0.63])
twist_slider = Slider(
    ax=axtwist,
    label="Twist",
    valmin=0,
    valmax=180,
    valinit=init_twist,
    orientation="vertical"
)
# updates coords off of csv file
def update(frame):
    x_coords, y_coords = extract_coords()
    line.set_xdata(x_coords)
    line.set_ydata(y_coords)
    fig.canvas.draw_idle()
#updates and calls robot values
previous_turn_value = turn_slider.val
previous_jaw_value = jaw_slider.val
previous_twist_value = twist_slider.val
def turn(val):
    global previous_turn_value
    current_turn_value = int(float(turn_slider.val))
    if current_turn_value != previous_turn_value:
        serialport.write(str(f"turn, {current_turn_value}\n").encode())
        previous_turn_value = current_turn_value
def twist(val):
    global previous_twist_value
    current_twist_value = int(twist_slider.val)
    if current_twist_value != previous_twist_value:
        serialport.write(str(f"twist, {current_twist_value}\n").encode())
        previous_twist_value = current_twist_value
def jaw(val):
    global previous_jaw_value
    current_jaw_value = int(jaw_slider.val)
    if current_jaw_value != previous_jaw_value:
        serialport.write(str(f"jaw, {current_jaw_value}\n").encode())
        previous_jaw_value = current_jaw_value
turn_slider.on_changed(turn)
jaw_slider.on_changed(jaw)
twist_slider.on_changed(twist)


# All the Buttons ----------------------------------------------------------------------------------------------------
# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')
def reset(event):
    turn_slider.reset()
    jaw_slider.reset()
    twist_slider.reset()
    draggable_point.reset()
button.on_clicked(reset)

# clearax = fig.add_axes([0.6, 0.025, 0.1, 0.04])
# button = Button(clearax, 'Reset', hovercolor='0.975')
# def clear(event):
#     print(None)
# button.on_clicked(reset)

# recordax = fig.add_axes([0.4, 0.025, 0.1, 0.04])
# button = Button(recordax, 'Record', hovercolor='0.975')
# def record(event):
#     print(None)
# button.on_clicked(reset)

# playax = fig.add_axes([0.2, 0.025, 0.1, 0.04])
# button = Button(playax, 'Play', hovercolor='0.975')
# def play(event):
#     print(None)
# button.on_clicked(reset)

#loop animation
ani = FuncAnimation(fig, update, frames=range(10), interval=100)
plt.show()
