# Tiny-Gimbal
Code and pcb for two axis self stabilizing servo gimbal using BNO055 and ATTiny16x4 for TJUAV

### V1 
Version for the See3Cam CU135, with 3 axis stabilization. Code was developed and used, but PCB was never manufactured

### V2.1
Due to poor results with the See3Cam, TJUAV has decided to switch to the Sony alpha cameras, specifically the A5100. However, all testing so far has been done with an A5000 camera we already have, which should be dimensionally identical to the A5100, although slower.

Yaw stabilization was removed, for the sake of weight saving and ease of cable management
The mechanism for the gimbal was completely overhauled, needing to be stronger for the camera being 400g instead of 20g
The gimbal was designed for the camera to be easily removeable this time around
A kill switch, and two control buttons (one for centering all servos, and one for putting the gimbal in a position where the camera is easily removeable) were added

Tested and working

### V2.2
Small feature improvements, including:
Zip tie points on the camera mount
Consolidating UART debug connectivity into the UPDI header using a SPDT DIP switch
LED indicators to tell whether the gimbal is centered, in remove position, and powered
U mount X frame instead of an L bracket, to support the camera from both sides and minimize lateral load on the X servo
Single bearing instead of dual bearing Y frame mechanism, to save mass
Sideways X servo, to save mass and size
Integrated buck converter mount on the spar clamp

Yet to be manufactured and tested
