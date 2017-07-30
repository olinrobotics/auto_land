# auto_land
Code for autonomous landing of a drone on a target

## Running the code

The simplest way to run the landing code is to run `vehicle_bringup.bash`, then fly over the target so the camera can see it and run `controller.launch`. Currently nothing is implemented to move to an estimated target position before the camera sees it.

## Nodes:
### `vehicle_bringup.bash` 
This is the main bringup file for communicating with the vehicle and setting up the TF frames which track the position of the vehicle and the target. It runs `apm.launch` and `vehicle.launch` and sets the pixhawk to stream all of the necessary data. It is a bash script instead of a launch file because `apm.launch` needs to be running for a few seconds to set up TF transforms before `vehicle.launch` will run.

### `fiducial_follower.py`
Uses a PID loop and velocity commands to align the `\fcu_aligned` and `\target` tf frames, where `fcu_aligned` is the tf frame with origin on the drone and axes aligned to the local coordinates for commands (see coordinates page of wiki).

Before running this code, you must run `vehicle_bringup.bash` and `pid.launch`

### `simple_follwer.py`
Moves the drone to the target using waypoint commands by sending the drone to the location of the target in the local coordinate frame based on the tf transform between `\local_origin` and `\target`

Before runing this code, you must run `vehicle_bringup.bash`

### TF managing code:
The set of TF frames in this code is somewhat confusing: There is a frame for the drone (`\fcu_utm`), given by mavros, which has transforms from a `\local_origin` to the vehicle, published at about 10 Hz. When you send velocity commands to the aircraft, however, they are in a north/south/up coordinate system, so I added another aircraft frame `\fcu_aligned` which has an origin in the same place as `fcu_utm` but has axes in the north/soutn/up directions. The camera has a `\camera` frame with a static transform to the airframe. The frame `\target_raw` is published relative to `\camera` when the camera sees the target, and is the raw output of the fiducial detection code. `\target` is a filtered version of `\target_raw`. It is published at a consistnt, faster rate, and is published as a transform from `\local_origin` to `\target`, which encodes an assumption that the target is stationary (at least over short timespans)

**`target_filter.py`**

Currently just republishes the last seen location of the target at a faster rate, this is where future code for target velocity estimation should go.


**`vehicle_tf.py`**

Manages the `\fcu_utm`, `\fcu_aligned` and `\camera` coordinate frames.

### Test Files:

**`test_airframe_tf.py` and `test_target_tf.py`**

For testing without a drone:

Airframe publishes the tf transforms that mavros would output, with random values

target publishes a random transform between `\camaera` and `\target_raw`

**`pixhawk_control_test.py`**

tests the functionality of sendig position and velocity commads to the pixhawk (also useufl for reference beuase it's pretty much the simplest code to send those commands)

## Launch Files:

### `vehicle.launch`
launches all of the code for detecting the target as well as for managing the tf frames

### `controller.launch`
launches `pid.launch` and runs `fiducial_follwer.py`

### `apm2.launch`
coppy of the launch file from the mavros package with a few things changed because the odroid is onboard

### `detector.launch`
Sets up and lauches the apriltags package to detect the landing target

### `pid.launch`
Launches the PID loops for controlling the drone

### `tf_manager.launch`
Launches all of the code to deal with the multiple tf frames (descirbed above)

### `test_tf.lauch`
Lauches the test tf scripts described above for testing code without an actual drone

## Target:

Two nested apriltags from the 25h9 family:
- Largest: ID 18, size: 6in
- Second: ID 11, size: 0.6in

note: eventually this will be scaled to 18in and a third tag will probably be added
