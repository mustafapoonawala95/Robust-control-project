# Robust-control-project

## SUMMARY
**In this project an output feedback system to track the vertical acceleration of a medium range air-to-air missile (MRAAM) is
designed and used in a True proportional navigation guidance law to intercept a target drone
with minimum miss distance. The missile’s short period dynamics are used for this purpose.
Robust servomechanism theory and a Luenberger observer is used in the design.   
The project is divided in 2 parts, A and B. Part A consists of designing the closed loop
system with the design requirements mentioned, using full state feedback first, and then using
output feedback with Acceleration and pitch rate as the outputs. Design charts are created at all significant points to get an intuition of the trends and aid in choosing the optimal gains.**

**In part B, the designed closed loop system is then incorporated in the guidance law and
minimum miss distance is evaluated. At this time changes in design are made if required.**

###  PART A (Autopilot Design)
#### Design requirements:
* At least 6 dB of gain and 35 degrees of phase margin.
* Maximum fin displacement of 35 degrees.
* Maximum fin rate of 350 degrees per second
* Loop gain at input crossover frequency less than 1/3 the actuator natural frequency.
* Percent undershoot less than 10%.
* 
<!---->
<!--Following are the open loop matrices provided.-->


