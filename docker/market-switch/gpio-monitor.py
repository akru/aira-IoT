from twisted.internet import reactor
from sysfs.gpio import Controller, INPUT, FALLING 
from urllib import urlopen
from datetime import timedelta, datetime

Controller.available_pins = [2, 3]

change_ts = datetime.now() 

def pin_changed(number, state):
    print("Pin '%d' changed to %d state" % (number, state))

    global change_ts
    print(datetime.now() - change_ts)
    if (datetime.now() - change_ts) < timedelta(0, 10):
        return
    else:
        change_ts = datetime.now()

    print("Timedelta filter passed")
    if number == 2:
        # RIGHT
        urlopen("http://192.168.0.254:3000/RIGHT").read()
    else:
        # LEFT
        urlopen("http://192.168.0.254:3000/LEFT").read()

Controller.alloc_pin(2, INPUT, pin_changed, FALLING)
Controller.alloc_pin(3, INPUT, pin_changed, FALLING)

if __name__ == '__main__':
    reactor.run()
