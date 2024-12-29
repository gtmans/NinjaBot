# NinjaBot
recreation of a Ninja Robot

![Dual_Base](https://github.com/gtmans/NinjaBot/blob/main/Dual_Base-small.jpg)

This code is an adaption of robotlk code and combines robotlk code Remote_XY (control by phone/app) and selfmove avoid code
You can use left and right arm button to turn selfmove avoid on or off
since I made a breakout board for the D1 using Dual Base v2 for LOLIN(WEMOS)
there are some port changes and an added Led (optional) to show if (battery)power is on or off  
The new led blinks depending on distance measurements
 
PwrLed            on Gpio 12 (D6)
ServoRightFootPin on Gpio 4  (D2) // you can use original port Gpio 2 (D4) but that interferes with buildin led and uploading code
ServoRightLegPin  on Gpio 5  (D1) // you can use original port
 
Because of these changes some of the 3D-parts were modified. You can download them at Github/gtmans.
You can use the original parts by making some ajustments (with saw and drill and take some parts away)
Also changed parts to make bigger servo-arms fit and added wheelcovers
