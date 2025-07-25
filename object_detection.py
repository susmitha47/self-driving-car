import RPi.GPIO as GPIO                    
import time

#Import time library
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)                   

TRIG = 17
ECHO = 27

m11=16
m12=12
m21=21
m22=20

GPIO.setup(TRIG,GPIO.OUT)                  
GPIO.setup(ECHO,GPIO.IN)                

GPIO.setup(m11,GPIO.OUT)
GPIO.setup(m12,GPIO.OUT)
GPIO.setup(m21,GPIO.OUT)
GPIO.setup(m22,GPIO.OUT)


time.sleep(2)

def stop():
    print('stop')
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)

def forward():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print('Forward')

def backward():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 1)
    print('back')

def left():
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print('left')

def right():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)
    print("right")



# Stop initially
stop()
time.sleep(1)
count=0
while True:
    avgDistance = 0
    
    # Measure distance 5 times and average
    for i in range(5):
        GPIO.output(TRIG, False)  
        time.sleep(0.1)  

        GPIO.output(TRIG, True)  
        time.sleep(0.00001)  
        GPIO.output(TRIG, False)  

        pulse_start = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        pulse_end = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start  
        distance = pulse_duration * 17150  
        distance = round(distance, 2)  
        avgDistance += distance

    avgDistance /= 5  # Get the average distance
    print(f"avgDistance: {avgDistance} cm")

    if avgDistance < 25:
        count += 1
        stop()
        time.sleep(1)
        
        backward()
        time.sleep(1.5)
        
        if count % 2 == 0:
            right()
        else:
            left()
            
        time.sleep(1.5)
        stop()
        time.sleep(1)
    else:
        forward() 
GPIO.cleanup()

