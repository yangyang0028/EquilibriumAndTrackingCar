import sensor, image, time
from pyb import UART, LED
import json
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 200)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()
uart = UART(3, 115200)
control_val=0
Roi1=[40,100,240,30]
Roi2=[0,190,320,30]
aaa=55
led=LED(1)
led.on()
while(True):
    x1=0;
    x2=0;
    ans_l_1=0
    ans_l_1_id=-2
    ans_l_2=0
    ans_l_2_id=-2
    img = sensor.snapshot(1.8)
    blobs1 = img.find_blobs([(0, 27, -128, 127, -128, 127)],roi=Roi1,pixels_threshold = 24,area_threshold = 5,merge = True)
    blobs2 = img.find_blobs([(0, 27, -128, 127, -128, 127)],roi=Roi2,pixels_threshold = 24,area_threshold = 5,merge = True)
    img.draw_rectangle(Roi1)
    img.draw_rectangle(Roi2)
    for b in blobs1:
        if ans_l_1<b.pixels():
            ans_l_1=b.pixels()
            ans_l_1_id=b

    for b in blobs2:
        if ans_l_2<b.pixels():
            ans_l_2=b.pixels()
            ans_l_2_id=b


    if ans_l_1 > 0:
        x = ans_l_1_id[0]
        y = ans_l_1_id[1]
        width = ans_l_1_id[2]
        height = ans_l_1_id[3]
        img.draw_rectangle([x,y,width,height])
        img.draw_cross(ans_l_1_id[5], ans_l_1_id[6])
        x1=ans_l_1_id[5]
    if ans_l_2 > 0:
        x = ans_l_2_id[0]
        y = ans_l_2_id[1]
        width = ans_l_2_id[2]
        height = ans_l_2_id[3]
        img.draw_rectangle([x,y,width,height])
        img.draw_cross(ans_l_2_id[5], ans_l_2_id[6])
        x2=ans_l_2_id[5]
    uart.write(aaa.to_bytes(1,'int')+x1.to_bytes(2,'int')+x2.to_bytes(2,'int'))
    print(x1,x2)
