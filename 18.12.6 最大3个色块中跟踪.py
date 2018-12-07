# 色块追踪 - By: Jack Zhang - 周三 12月 5 2018

import sensor, image, time

from pid import PID
from pyb import Servo
from pyb import LED

pan_servo=Servo(1)
tilt_servo=Servo(2)

DISTANCE_THRESHOLD = 100
WAIT_THRESHOLD = 50
MOVE_THRESHOLD = 3
forget_ratio = 0.3
target_blob = []
old_x = -1
old_y = -1
count = 0

pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

def auto_get_colour():
# Capture the color thresholds for whatever was in the center of the image.
    r = [(320//2)-(50//2), (240//2)-(50//2), 50, 50] # 50x50 center of QVGA.

    print("Auto algorithms done. Hold the object you want to track in front of the camera in the box.")
    print("MAKE SURE THE COLOR OF THE OBJECT YOU WANT TO TRACK IS FULLY ENCLOSED BY THE BOX!")
    #LED(1).on()
    #time.sleep(500)
    #LED(1).off()
    #time.sleep(500)
    #LED(1).on()
    #time.sleep(500)
    #LED(1).off()
    #time.sleep(500)
    #LED(1).on()
    #time.sleep(500)
    #LED(1).off()
    #time.sleep(500)

    for i in range(100):
        img = sensor.snapshot()
        img.draw_rectangle(r)

    print("Learning thresholds...")
    threshold = [50, 50, 0, 0, 0, 0] # Middle L, A, B values.
    for i in range(100):
        img = sensor.snapshot()
        hist = img.get_histogram(roi=r)
        lo = hist.get_percentile(0.01) # Get the CDF of the histogram at the 1% range (ADJUST AS NECESSARY)!
        hi = hist.get_percentile(0.99) # Get the CDF of the histogram at the 99% range (ADJUST AS NECESSARY)!
        # Average in percentile values.
        threshold[0] = (threshold[0] + lo.l_value()) // 2
        threshold[1] = (threshold[1] + hi.l_value()) // 2
        threshold[2] = (threshold[2] + lo.a_value()) // 2
        threshold[3] = (threshold[3] + hi.a_value()) // 2
        threshold[4] = (threshold[4] + lo.b_value()) // 2
        threshold[5] = (threshold[5] + hi.b_value()) // 2
        for blob in img.find_blobs([threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10):
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_rectangle(r)

    print("Thresholds learned...")
    print("Tracking colors...")
    #LED(3).on()
    #time.sleep(500)
    #LED(3).off()
    #time.sleep(500)
    #LED(3).on()
    #time.sleep(500)
    #LED(3).off()
    #time.sleep(500)
    #LED(3).on()
    #time.sleep(500)
    #LED(3).off()
    #time.sleep(500)
    return threshold

def find_max3(blobs):
    size=[]
    for x in blobs:
        size.append(x[4])
    temp=[]
    new=[]
    Inf = 0
    for i in range(3):
        temp.append(size.index(max(size)))
        size[size.index(max(size))]=Inf
        new.append(blobs[temp[i]])
    return new

def select_blobs(blobs, old_blob):
# 输入：blobs
# 输出：改变old_x,old_y全局变量
#      返回目标色块
    global old_x, old_y, count
    #用到的变量：
    #target_blob = []#目标色块
    #candi = 0       #候选下标
    #min_dis = 0     #最小距离
    candi = 0

    blobs = find_max3(blobs)
    min_dis = DISTANCE_THRESHOLD
    if old_x == -1 and old_y == -1:
    #初始化跟踪对象
        print('Init!!')
        old_blob = blobs[0]
        old_x = old_blob.cx()
        old_y = old_blob.cy()
        return old_blob
    old_x = old_blob.cx()
    old_y = old_blob.cy()
    for i in range(len(blobs)):
    #找出最小距离色块
        distance = abs(blobs[i].cx()-old_x) + abs(blobs[i].cy()-old_y)
        img.draw_rectangle(blobs[i].rect()) # rect
        if distance < min_dis:
            min_dis = distance
            candi = i
        print(blobs[i],distance,min_dis)



    if abs(min_dis) >=DISTANCE_THRESHOLD and count <WAIT_THRESHOLD:
    # 距离过大，认为是暂时失去目标，等待30帧。否则跟丢，找距离上一个最近的
        print('N')
        count = count+1
        target_blob = old_blob
    else:
        print('Y')
        count = 0
        target_blob = blobs[candi]

    print(old_x,old_y)
    print(target_blob.cx(),target_blob.cy(),min_dis)
    img.draw_rectangle(target_blob.rect()) # rect
    img.draw_cross(target_blob.cx(),target_blob.cy()) # cx, cy
    return target_blob

red_threshold = auto_get_colour()
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    blobs = img.find_blobs([red_threshold], area_threshold = 100, merge=True)
    #找到视野中的线,merge=true,将找到的图像区域合并成一个
    if blobs:
        target_blob = select_blobs(blobs, target_blob)
        pan_error = target_blob.cx()-img.width()/2
        tilt_error = target_blob.cy()-img.height()/2

        print(red_threshold)
        print("pan_error: ", pan_error)
        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        print("pan_output",pan_output)
        pan_servo.angle(pan_servo.angle()+pan_output)
        tilt_servo.angle(tilt_servo.angle()-tilt_output)

