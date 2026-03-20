Performance
===========

Summary
--------
On demo day, BB made it to all five checkpoints on two of the three runs. The other run it made it to four of the five. It moved slowly compared to other teams robots, but the logic and methods implemented were repeatable and fairly reliable.

Video
-----
The following video shows BB completing the course on demo day.

.. video:: _static/videos/run.mp4
   :width: 600
   :align: center

Issues / Room for Improvement
-----------------------------
Most of the issues our team encountered were related to tuning calibration rather than logic. It took time to determine the appropriate gains and setpoint speed for line-following the on the squiggly segment of the track. We also ran into memory shortage issues. Running the garbage collector and removing unnecessary queues helped clear space and maximize memory efficiency. 

Given more time, we would have sought to make BB complete the course faster and implement more states to complete the bonus cup challenges. We also would have configured a bluetooth module for wireless startup and UI. It also would have been interesting to experiment with the imu and observer outputs. We did not do extensive testing using the state estimator as the feedback for position control, but it would be an interesting experiment.