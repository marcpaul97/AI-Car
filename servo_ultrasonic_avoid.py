#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time


#Definition of  motor pins 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of  key
key = 8

#Definition of  ultrasonic module pins
EchoPin = 0
TrigPin = 1

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

#Definition of servo pin
ServoPin = 23


#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #The first tracking infrared sensor pin on the left is connected to  BCM port 3 of Raspberry pi
TrackSensorLeftPin2  =  5   #The second tracking infrared sensor pin on the left is connected to  BCM port 5 of Raspberry pi
TrackSensorRightPin1 =  4    #The first tracking infrared sensor pin on the right is connected to  BCM port 4 of Raspberry pi
TrackSensorRightPin2 =  18   #The second tracking infrared sensor pin on the right is connected to  BCMport 18 of Raspberry pi

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Ultrasonic pin,RGB pin,servo pin initialization
class Car():
    def __init__(self, start):
        self.start = start
        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(key,GPIO.IN)
        GPIO.setup(EchoPin,GPIO.IN)
        GPIO.setup(TrigPin,GPIO.OUT)
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)
        GPIO.setup(ServoPin, GPIO.OUT)
        GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
        GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
        GPIO.setup(TrackSensorRightPin1,GPIO.IN)
        GPIO.setup(TrackSensorRightPin2,GPIO.IN)
        #Set the PWM pin and frequency is 2000hz
        self.pwm_ENA = GPIO.PWM(ENA, 2000)
        self.pwm_ENB = GPIO.PWM(ENB, 2000)
        self.pwm_ENA.start(0)
        self.pwm_ENB.start(0)
    
        self.pwm_servo = GPIO.PWM(ServoPin, 50)
        self.pwm_servo.start(0)
        
    #advance
    def run(self,leftspeed, rightspeed, q):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(1)

    
    #back
    def back(self,leftspeed, rightspeed, q):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(1)

        
    #turn left
    def left(self,leftspeed, rightspeed, q):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(1)

    
    #trun right 
    def right(self,leftspeed, rightspeed, q):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(1)

        
    #turn left in place
    def spin_left(self,leftspeed, rightspeed, q):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(1)

    
    #turn right in place
    def spin_right(self,leftspeed, rightspeed, q):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(1)
 
    
    #brake
    def brake(self, q):
       GPIO.output(IN1, GPIO.LOW)
       GPIO.output(IN2, GPIO.LOW)
       GPIO.output(IN3, GPIO.LOW)
       GPIO.output(IN4, GPIO.LOW)
       time.sleep(1)
  
    
    #Button detection
    def key_scan(self):
        while GPIO.input(key):
            pass
        while not GPIO.input(key):
            time.sleep(0.01)
            if not GPIO.input(key):
                time.sleep(0.01)
                while not GPIO.input(key):
                       pass
                    
    #Ultrasonic function
    def Distance_test(self, q):
        GPIO.output(TrigPin,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(TrigPin,GPIO.LOW)
        while not GPIO.input(EchoPin):
            pass
        t1 = time.time()
        while GPIO.input(EchoPin):
            pass
        t2 = time.time()
        #print("distance is %d " % (((t2 - t1)* 340 / 2) * 100))
        time.sleep(.25)
        q.put(((t2 - t1)* 340 / 2) * 100)
        time.sleep(1)
        return
        
    #The servo rotates to the specified angle
    def servo_appointed_detection(self,pos, q):
        #for i in range(18):
        self.pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)
        time.sleep(1)
 
            
            
    def doTrack(self, bolA, q ): #Checks all 4 tracks to see if anyone of them are directly under black, which is a road in our scenario
        TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
        TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
        leftV = TrackSensorLeftValue1
        middleLeftV = TrackSensorLeftValue2
        middleRightV = TrackSensorRightValue1
        rightV = TrackSensorRightValue2
        bolA = [0, 0, 0, 0]
        bolA[0] = leftV
        bolA[1] = middleLeftV
        bolA[2] = middleRightV
        bolA[3] = rightV
        time.sleep(1)
        
   