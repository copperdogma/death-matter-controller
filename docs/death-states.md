# Death States

These states were gathered from the spec.md doc in the Death project:

These are the states we want to be able to send from this Matter controller over UART to the ESP32-WROVER running the Death skull.

FAR_MOTION_DETECTED - Simulate the FAR motion sensor being triggered.
NEAR_MOTION_DETECTED - Simulate the NEAR motion sensor being triggered.
PLAY_WELCOME - Playing welcome skit (FAR sensor)
WAIT_FOR_NEAR - Waiting for NEAR sensor after welcome
PLAY_FINGER_PROMPT - Playing "put finger in my mouth" skit
MOUTH_OPEN_WAIT_FINGER - Mouth open, waiting for finger
FINGER_DETECTED - Finger detected, waiting for snap delay
SNAP_WITH_FINGER - Snap with finger (play finger snap skit)
SNAP_NO_FINGER - Snap without finger (play no finger skit)
FORTUNE_FLOW - Fortune flow sequence
FORTUNE_DONE - Fortune complete, play goodbye skit
COOLDOWN - Post-fortune cooldown period


Note: FAR_MOTION_DETECTED and NEAR_MOTION_DETECTED are the only ones that will be sent when in normal use. The user will add the Near and Far Matter occupancy sensors to Apple Home, then add the Death Matter controller to Apple Home, then set up the automation so that when the Far occupancy sensor is triggered it turns on the NEAR_MOTION_DETECTED "power button" on the Death controller, and similar for FAR.