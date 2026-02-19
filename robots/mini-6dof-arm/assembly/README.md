# Assembly

Videos: https://drive.google.com/drive/folders/1Yrs1OJ-STuqMprF4RufwyShn4Wvzg0KU

## Power connection

https://docs.google.com/document/d/1Wie1Qy5fnm7ER6EPi7TEyTMOaUlAVmLpw7wHglr-cPQ/edit?tab=t.0#heading=h.1sr0lyl94v1s

The software instructions are outdated, use USB cable from the Pi to the servo controller and start from the QUICK_START.md document.

CAUTION: If you keep the arm constantly powered, the elbow servos might burn out from the constant load

## Printable parts

Step files can be found in:
https://drive.google.com/drive/folders/1Zv-EZmE0popnsXJyIiYnnzl3_a3QGW4C

## Servo motors

Feetech URT-1 servo controller
Feetech STS3215-12v 30kg servos
Amazon link - https://www.amazon.com/dp/B0F38DS2QT
Servo software (Windows)
https://www.feetechrc.com/software.html
Software-深圳飞特模型有限公司
get the FD 1.9.8.3

Servo controller software (Mac OS, Linux):
https://github.com/iotdesignshop/Feetech-tuna/tree/main

Alternative JS software:
https://bambot.org/feetech.js

Joint - servo id mapping:
J1 (base rotate) - servo id 10
J2 (shoulder bend) - servo id 20, 21
J3 (elbow bend) - servo id 30, 31
J4 (forearm rotate) - servo id 40
J5 (wrist bend) - servo id 50
J6 (wrist rotate) - servo id 60
J7 (gripper) - servo id 100

Note: J1 has 1:2 reduction, so 45 degrees control is 90 degrees of rotation.

## Premade disk image

If you prefer premade Pi disk image, download from here:
https://drive.google.com/file/d/1rXxFpzfOMQghmZEDju51e3wvshWYfiML/view?usp=drive_link

Login details:

username: gradientrobotics
password: password
