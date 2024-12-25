speed_left = 0
speed_right = 0
angleLfront = 0
angleRfront = 0
angleLback = 0
angleRback = 0
joint1 = 90
joint2 = -175
joint0 = 0

def forward():
    global speed_left, speed_right
    speed_left += 10
    speed_right += 10
    if speed_left > 100:
        speed_left = 100
    if speed_right > 100:
        speed_right = 100

def backward():
    global speed_left, speed_right
    speed_left -= 10
    speed_right -= 10
    if speed_left < -100:
        speed_left = -100
    if speed_right < -100:
        speed_right = -100

def left():
    global angleLfront, angleRfront, angleLback, angleRback
    angleLfront -= 10
    angleRfront -= 10
    angleLback += 10
    angleRback += 10
    if angleLfront < -50:
        angleLfront = -50
    if angleRfront < -50:
        angleRfront = -50
    if angleLback > 50:
        angleLback = 50
    if angleRback > 50:
        angleRback = 50

def right():
    global angleLfront, angleRfront, angleLback, angleRback
    angleLfront += 10
    angleRfront += 10
    angleLback -= 10
    angleRback -= 10
    if angleLfront > 50:
        angleLfront = 50
    if angleRfront > 50:
        angleRfront = 50
    if angleLback < -50:
        angleLback = -50
    if angleRback < -50:
        angleRback = -50

def spin_left():
    global speed_left, speed_right, angleLfront, angleRfront, angleLback, angleRback
    speed_left = -70
    speed_right = 70
    angleLfront = 45
    angleRfront = -45
    angleLback = -45  
    angleRback = 45  

def spin_right():
    global speed_left, speed_right, angleLfront, angleRfront, angleLback, angleRback
    speed_left = 70
    speed_right = -70
    angleLfront = 45
    angleRfront = -45
    angleLback = -45  
    angleRback = 45    

def stop():
    global speed_left, speed_right
    speed_left = 0
    speed_right = 0

def stop_spin():
    global speed_left, speed_right, angleLfront, angleRfront, angleLback, angleRback
    speed_left = 0
    speed_right = 0
    angleLfront = 0
    angleRfront = 0
    angleLback = 0
    angleRback = 0

def joint1_down():
    global joint1
    joint1 -= 5
    if joint1 < -90:
        joint1 = -90

def joint1_up():
    global joint1
    joint1 += 5
    if joint1 > 90:
        joint1 = 90

def joint2_down():
    global joint2
    joint2 -= 5
    if joint2 < -175:
        joint2 = -175

def joint2_up():
    global joint2
    joint2 += 5
    if joint2 > 175:
        joint2 = 175

def joint0_left():
    global joint0
    joint0 -= 5
    if joint0 < -180:
        joint0 = -180

def joint0_right():
    global joint0
    joint0 += 5
    if joint0 > 180:
        joint0 = 180