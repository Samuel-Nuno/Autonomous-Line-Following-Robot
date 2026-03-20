from pyb import Timer
from pyb import Pin
import utime
from task_share import Share, Queue, show_all
from cotask import Task, task_list
from gc import collect

from task_line_sensor import task_line_sensor
from linesensor import linesensor
from task_bump import task_bump

from ClosedLoop import Closed_Loop
from task_user import task_user

from left_drive_system import task_left_drive
from right_drive_system import task_right_drive
from task_start_button import task_start_button

# PWM timer (both motors on Timer 8)
tim_pwm = Timer(8, freq=20000)

# Shares
leftMotorGo  = Share("B", name="Left Go")
rightMotorGo = Share("B", name="Right Go")

spL = Share("f", name="Setpoint L")
spR = Share("f", name="Setpoint R")

vL = Share("f", name="Velocity L")
vR = Share("f", name="Velocity R")

# NEW: encoder position shares
posL = Share("l", name="Position L")
posR = Share("l", name="Position R")

effL = Share("h", name="Effort L")
effR = Share("h", name="Effort R")

Kp = Share("f", name="Kp")
Ki = Share("f", name="Ki")

# Stronger defaults so it actually moves
Kp.put(0.2)
Ki.put(0)

# Line following shares
lf_enable   = Share("B", name="LF Enable")
lf_forwardV = Share("f", name="LF Forward Vel")
lf_centroid = Share("f", name="LF Centroid")
bump_pressed = Share("B", name="Bump Pressed")

# defaults
lf_enable.put(0)
lf_forwardV.put(250.0)
leftMotorGo.put(0)
rightMotorGo.put(0)
spL.put(0)
spR.put(0)
effL.put(0)
effR.put(0)
lf_centroid.put(-1.0)
bump_pressed.put(0)
posL.put(0)
posR.put(0)

# Queues
# Keep moderate to avoid RAM trouble
N_SAMPLES = 100
N_CENTROID_SAMPLES = 100
dataValuesL = Queue("f", N_SAMPLES, name="Left Velocity Buffer")
dataValuesR = Queue("f", N_SAMPLES, name="Right Velocity Buffer")
timeValuesL = Queue("L", N_SAMPLES, name="Left Time Buffer")
timeValuesR = Queue("L", N_SAMPLES, name="Right Time Buffer")
lf_centroid_q = Queue("f", N_CENTROID_SAMPLES, name="Centroid Buffer")

# Tasks
leftTask = task_left_drive(
    tim_pwm, leftMotorGo, effL, vL, dataValuesL, timeValuesL,
    posL_share=posL,
    lf_centroid_share=lf_centroid,
    lf_centroid_queue=lf_centroid_q
)

rightTask = task_right_drive(
    tim_pwm, rightMotorGo, effR, vR, dataValuesR, timeValuesR,
    posR_share=posR
)

ctrlTask = Closed_Loop(
    spL, spR, vL, vR, effL, effR, Kp, Ki,
    lf_enable_share=lf_enable,
    lf_v_share=lf_forwardV,
    lf_centroid_share=lf_centroid,
    bump_share=bump_pressed,
    posL_share=posL,
    posR_share=posR,
    lf_deadband=0.4,
    lf_dv_turn=180,
    lf_lost_val=-1
)

# Line sensor task
qtr_adc_pins = [
    Pin.cpu.A0,
    Pin.cpu.A1,
    Pin.cpu.A4,
    Pin.cpu.B0,
    Pin.cpu.C1,
    Pin.cpu.C0,
    Pin.cpu.C2,
    Pin.cpu.C3,
]
lineTask = task_line_sensor(qtr_adc_pins, lf_centroid, lf_enable)

bumpTask = task_bump(bump_pressed, pin=Pin.cpu.B12, active_low=True, debounce_count=3)

uiTask = task_user(
    leftMotorGo, rightMotorGo,
    spL, spR,
    effL, effR,
    dataValuesL, dataValuesR,
    timeValuesL, timeValuesR,
    Kp_share=Kp,
    Ki_share=Ki,
    lf_enable_share=lf_enable,
    lf_v_share=lf_forwardV,
    lf_centroid_queue=lf_centroid_q,
    sensor=lineTask.sensor,
)

startTask = task_start_button(
    leftMotorGo,
    rightMotorGo,
    lf_enable=lf_enable,
    lf_v_share=lf_forwardV,
    effL=effL,
    effR=effR,
    spL=spL,
    spR=spR,
    start_speed=220.0
)

# Scheduler
task_list.append(Task(leftTask.run,  name="Left Drive",  priority=2, period=20, profile=False))
task_list.append(Task(rightTask.run, name="Right Drive", priority=2, period=20, profile=False))
task_list.append(Task(ctrlTask.run,  name="Closed Loop", priority=3, period=20, profile=False))
task_list.append(Task(lineTask.run,  name="Line Sensor", priority=2, period=20, profile=False))
task_list.append(Task(bumpTask.run,  name="Bump",        priority=2, period=10, profile=False))
task_list.append(Task(uiTask.run,    name="User UI",     priority=1, period=0,  profile=False))
task_list.append(Task(startTask.run, name="Start Button", priority=1, period=50, profile=False))

collect()

while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        print("Program Terminating")
        break

print(task_list)
