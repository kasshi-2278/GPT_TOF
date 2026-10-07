"""Photographic transcription, NOT the original user-supplied .py file.

Reconstructed from the five photos in this conversation. Decision order,
operator precedence and reverse sleeps retained. Do not treat this as a
validated safe controller. Load ONLY via RC Simulator V5 for testing.

WARNING: direct execution on a real Raspberry Pi accesses hardware.
"""
import time
import RPi.GPIO as GPIO
import Adafruit_PCA9685
GPIO.setmode(GPIO.BOARD)
GPIO.cleanup()
t_list=[15,13,35,32,36]
e_list=[33,40,37,31,38]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(e_list,GPIO.IN)
pwm=Adafruit_PCA9685.PCA9685(address=0x40)
pwm.set_pwm_freq(60)
STEERING_CHANNEL=1
STEERING_RIGHT_PWM=330
STEERING_CENTER_PWM=435
STEERING_LEFT_PWM=540
THROTTLE_CHANNEL=7
THROTTLE_FORWARD_PWM=500
THROTTLE_STOPPED_PWM=375
THROTTLE_REVERSE_PWM=220
pwm.set_pwm(THROTTLE_CHANNEL,0,THROTTLE_STOPPED_PWM)
pwm.set_pwm(STEERING_CHANNEL,0,STEERING_CENTER_PWM)
input('Press Enter to start...')

def Mesure(GPIO,time,trig,echo):
    sigoff=0
    sigon=0
    GPIO.output(trig,GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig,GPIO.LOW)
    kijyun=time.time()
    while GPIO.input(echo)==GPIO.LOW:
        sigoff=time.time()
        if sigoff-kijyun>0.02:
            break
    while GPIO.input(echo)==GPIO.HIGH:
        sigon=time.time()
        if sigon-sigoff>0.02:
            break
    return (sigon-sigoff)*34000/2

t=time.time()
try:
    while True:
        Fr=Mesure(GPIO,time,15,33)
        FrLh=Mesure(GPIO,time,13,40)
        FrRh=Mesure(GPIO,time,32,31)
        RrLh=Mesure(GPIO,time,35,37)
        RrRh=Mesure(GPIO,time,36,38)
        print(f'Fr {Fr:.1f}; FrLh {FrLh:.1f}; FrRh {FrRh:.1f}; RrLh {RrLh:.1f}; RrRh {RrRh:.1f}')
        t_before=t
        t=time.time()
        print(f'1サイクル処理時間: {t-t_before:.3f}秒')
        Accel=40
        Handle=0
        comment='(1) 直進'
        if RrLh<30 or FrLh<80:
            Accel=40
            Handle=-50
            comment='(2) 少し右折'
        if RrRh<30 or FrRh<80:
            Accel=40
            Handle=50
            comment='(3) 少し左折'
        if Fr<100 and RrRh+FrRh>RrLh+FrLh:
            Accel=30
            Handle=-100
            comment='(4) だいぶ右折'
        if Fr<100 and RrRh+FrRh<=RrLh+FrLh:
            Accel=30
            Handle=100
            comment='(5) だいぶ左折'
        # Python evaluates AND before OR; do not regroup these conditions.
        if Fr<35 or FrRh<35 and min(FrRh,RrRh)<=min(FrLh,RrLh):
            Accel=-50
            Handle=-40
            comment='(6) 後退の右旋回'
        if Fr<35 or FrLh<35 and min(FrRh,RrRh)>min(FrLh,RrLh):
            Accel=-50
            Handle=40
            comment='(7) 後退の左旋回'
        if Accel>0:
            throttle_pwm=int(THROTTLE_STOPPED_PWM+(THROTTLE_FORWARD_PWM-THROTTLE_STOPPED_PWM)*Accel/100)
            pwm.set_pwm(THROTTLE_CHANNEL,0,throttle_pwm)
        elif Accel==0:
            pwm.set_pwm(THROTTLE_CHANNEL,0,THROTTLE_STOPPED_PWM)
            time.sleep(.01)
        else:
            throttle_pwm=int(THROTTLE_STOPPED_PWM+(THROTTLE_STOPPED_PWM-THROTTLE_REVERSE_PWM)*Accel/100)
            pwm.set_pwm(THROTTLE_CHANNEL,0,throttle_pwm)
            time.sleep(.2)
            pwm.set_pwm(THROTTLE_CHANNEL,0,THROTTLE_STOPPED_PWM)
            time.sleep(.02)
            pwm.set_pwm(THROTTLE_CHANNEL,0,throttle_pwm)
            time.sleep(.3)
        if Handle<=0:
            steer_pwm=int(STEERING_CENTER_PWM+(STEERING_CENTER_PWM-STEERING_RIGHT_PWM)*Handle/100)
        else:
            steer_pwm=int(STEERING_CENTER_PWM+(STEERING_LEFT_PWM-STEERING_CENTER_PWM)*Handle/100)
        pwm.set_pwm(STEERING_CHANNEL,0,steer_pwm)
        print(comment,'Handle=',Handle,'Accel=',Accel)
except KeyboardInterrupt:
    pwm.set_pwm(THROTTLE_CHANNEL,0,THROTTLE_STOPPED_PWM)
    pwm.set_pwm(STEERING_CHANNEL,0,STEERING_CENTER_PWM)
    GPIO.cleanup()
