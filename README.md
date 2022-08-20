# QSTP_Aerial-Robotics_2021

Week4: Sensors are inaccurate. This is the motivation behind a huge body of work in filtering, and the algorithm used for this purpose is the G-H filter.
       The algorithm goes as follows:
       
       Initialization
       
        1. Initialize the state of the filter
        2. Initialize our belief in the state
        
       Predict
       
        1. Use system behavior to predict state at the next time step
        2. Adjust belief to account for the uncertainty in prediction

       Update

        1. Get a measurement and associated belief about its accuracy
        2. Compute residual between estimated state and measurement
        3. New estimate is somewhere on the residual line




