import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
timeout = 0.020
spin=7
while 1:
        GPIO.setup(spin, GPIO.OUT)
        #cleanup output
        GPIO.output(spin, 0)

        time.sleep(0.000002)

        #send signal
        GPIO.output(spin, 1)

        time.sleep(0.000005)

        GPIO.output(spin, 0)

        GPIO.setup(spin, GPIO.IN)
        
        goodread=True
        watchtime=time.time()
        while GPIO.input(spin)==0 and goodread:
                starttime=time.time()
                if (starttime-watchtime > timeout):
                        goodread=False

        if goodread:
                watchtime=time.time()
                while GPIO.input(spin)==1 and goodread:
                        endtime=time.time()
                        if (endtime-watchtime > timeout):
                                goodread=False
        
        if goodread:
                duration=endtime-starttime
                distance=duration*34000/2
                print distance  