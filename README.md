# crover
Challenge task

Assumptions:
Assumed all inputs are in metres, metres per sec, radians, radians/sec

Method:

1. I started off by first having a look at the sampling intervals of both the odometer and GPS data to get a first insight.
2. Then I wanted to do a propagation check by taking the initial state from the truth data, propagate using the odometer 
readings and compare trajectory with the truth. That did not fare so well.
3. Then I did the same, but this time using the true speeds from the the truth data, this did not fare well either
4. Unfortunately, I did not have more time to work on the problem anymore. 

Proposal:
I wanted to implement a simple Kalman filter to tackle the problem.
I expect the behaviour to be that every time I get a GPS update the uncertainty in my state would go down and then grow (assuming odometer readings
to be not too reliable when it comes to predicting the state) until the next GPS reading. Also, the orientation uncertainty would be reduced by use
of GPS data due to the indirect relationship of relating linear speeds in the car frame to the map (inertial) frame.

Running the Python script:
I chose Python as I did not need anything more than numpy and matplotlib and since it is is the language that I am currently using at work
To run the script, simply type <file_location>localisation.py in the command line/terminal
