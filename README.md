Run the drone.py file to see the uncertainty of the drone change from t=1 t=5.
The time is controlled by the line 94, N which changes what the number of time steps should be

Drone.py also includes the code that adds the measurement field and incorporates it. Using the measurement you can find the original sigma and the predicted new sigma after a random measurement at t=7. Uncomment everything after line 102 and change N to 6 in order to have the sigma measurement added at time = 7


Drone2.py has the inclusion of probability into the measurements.

Drone3.py is the full implementation of the kalman filter. You can change prob to what is desired, as well as time time simulated by changing ii

