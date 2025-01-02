# NinjaBot
recreation of a Ninja Robot

<img src="https://github.com/gtmans/NinjaBot/blob/main/Ninja_top_view.JPG" width="300" align="right" />
<img src="https://github.com/gtmans/NinjaBot/blob/main/Ninja_front_view.JPG" width="300" align="left" />

This code and 3D model is an adaption of robotlk code and 3D-parts and combines robotlk code Remote_XY (control by phone/app) and selfmove avoid code.
You can use left and right arm button to turn selfmove avoid on or off.
I made a breakout board for the D1 using Dual Base v2 for LOLIN(WEMOS)
<BR><BR>
<img src="https://github.com/gtmans/NinjaBot/blob/main/Dual_Base-empty-small.jpg" width="275" />
<BR><BR><BR>
There are some port changes (made possible by the breakoutboard) and an added Led (optional) to show if (battery)power is on or off. The led blinks depending on distance measurements
<BR><BR> 
PwrLed            on Gpio 12 (D6)
ServoRightFootPin on Gpio 4  (D2) // you can use original port Gpio 2 (D4) but that interferes with buildin led and uploading code
ServoRightLegPin  on Gpio 5  (D1) // you can use original port
 
Because of these changes some of the 3D-parts were modified. You can download them at Github/gtmans/NinjaBot/3D.
You can use the original parts by making some ajustments (with saw and drill and take some parts away)
Also changed parts to make bigger servo-arms fit and added wheelcovers. Changed wheels for "tires" made of plastic drainage tube diameter 6mm. Changed the switch-hole in main body part to fit 7x7 mm. switch and changed position of USB-C hole fot the D1 on its base.

<img src="https://github.com/gtmans/NinjaBot/blob/main/Dual%20base%20breakout2_bb.png" width="600" />

Original description and source: https://robotlk.com/how-to-make-a-ninja-robot/
