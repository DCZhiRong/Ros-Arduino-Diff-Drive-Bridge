# Ros-Arduino-Diff-Drive-Bridge
Just some Arduino code for communicating with a fork of [Josh Newans's diffdrive arduino library](https://github.com/joshnewans/diffdrive_arduino) (I hope this is how to spell your name)

### Hardware specifications
The arduino code here assumes you are using an ams as5048b with a cytron motor driver.

The following external libraries are being used:
- [sosandroid's ams as5048b library](https://github.com/sosandroid/AMS_AS5048B)
- [cytron motor driver library](https://github.com/CytronTechnologies/CytronMotorDriver)

## How to use
The arduino code takes in serial commands and parses them into motor speed controls
The code takes in serial commands using this format:
```
"mode" "motor1 arg" "motor2 arg"
```

An example would be like:
```
e 20 20
```

## Modes
### e - target speed mode
Motor args are in radians per sec and are multiplied by 1o0.
- Note that negative motor args cause the motors to reverse their direction
- If either one or both motor args aren't sent the program will just assume their value is 0

Setting a motor speed of 1 rad/s would look like this
```
e 100 100
```
and would return
```
OK
```
If everything is fine


### p - raw pwm mode
Motor args are in 8 bit integers and directly control the pwm level sent to the motor driver.
- Note that negative motor args cause the motors to reverse their direction
- If either one or both motor args aren't sent the program will just assume their value is 0

Setting a pwm value of 255 would look like this
```
p 255 255
```
and would return
```
OK
```
If everything is fine


## s - returns the measured speed at the time
Using it would look like this
```
s 
```
and would return motor speed in rads/s
```
1.0
```
