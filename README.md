# Fetch wireless runstop
## Do not use this as an emergency stop
This should not be considered an emergency stop or used for anything safety-critical. This is not a rigorous way to implement an emergency stop as described by industrial standards such as ISO 13850. This is intended to provide a similar or slightly improved level of protection to the runstop provided by the PS4 controller bundled with a Fetch robot.

Compared to the PS4 controller runstop as configured by default with Fetch robots, this has the added benefit that it will engage the runstop should the wireless device disconnect. However, if you need something actually robust and safety-critical I suggest looking into off-the-shelf industrial wireless e-stop systems that trip the breakers on the robot electrically rather than through software.

## How to use this
This package implements a wireless runstop button for a Fetch robot that operates over TCP. The ROS node connects to the runstop button over a socket that assumes that the runstop will continuously send packets with the byte `0` as long as the runstop is disengaged (and `1` otherwise, although it will stop if any other packet is received).

If the node does not receive a packet within a timeout (by default this is 2 seconds), it will engage the runstop for safety.

Since this node is intended to operate the software runstop for a Fetch robot, it by default publishes `True` or `False` to the `enable_software_runstop` topic (this can of course be remapped).

The following parameters are used:
|Parameter|Default|Description|
|---------|-------|-----------|
|`host`|`fetch-estop`|The hostname for the wireless e-stop|
|`port`|`1337`|The port for the wireless e-stop|
|`timeout`|`2`|The timeout in seconds after which the software runstop will be engaged if no packet is received from the e-stop button|

## ESP2866 code
There is a reference implementation for the e-stop code written in the Arduino language in `wireless_runstop.ino`. This is intended to be flashed to an ESP8266. Configure the parameters `#define`d in the beginning of the file for your local environment.