# How-To Operate
This document should be a short introduction into the usage of the ARL Upper Limb Robot. 
It will contain general information about the control system and also examples how to interact
and use the robot.
This document describes the robot running with a Raspberry Pi 3.

## Introduction to the Setup
The introduction to the robot's setup as a whole should lead to an easier understanding about how the individual parts interact with each other.

### Technical Background

#### Controllers
Pictures of the Raspberry Pi's actual pin usage and the pin order on the individual controller can be found within the GitHub project containing all [schematics](https://github.com/arne48/rasp-pi_extension). 
[Raspberry Pi Extensions](https://github.com/arne48/rasp-pi_extension/blob/master/Info/pin_usage.odp)

1. __The Air Valve Controller (Activation)__
The Controller for the contineouse air valves _(Festo MPYE-5-M5)_   is a AD5360 DAC from Analog Devices.
This DAC provides a voltage range of __ -10V ... 10V__ in _65536_ steps.
_Festo's MPYE-5-M5_ puts its actuation range from blow-off to completely open into the range of _0V... 10V_ which leaves the AD5360 with 32768 usable steps for setting the valve's position. 
This range is mapped in within the controllers to a normalized mapping from _-1 ... 1_

2. __The Pressure Sensor Controller (Pressure)__
Analog Device's AD7616 ADC is used for reading out each muscle's pressure sensor _(SMC PSE540-R04)_.
SMC's pressure sensor provides a range of measurement from _0 ... 1MPa_ with a resolution of _16-bit_.
A calibration which would map the ADC's raw measurment to _bar_ or _psi_ is not applied.
The muscles of this robot commonly are acting within a range of _3360 ... 8500_

3. __The Load Cell Controller (Tension)__
The load cells of the tension sensors are read out using the AD7730 of Analog Devices. One module for 16 muscles is populated with 8 of this bridge tranceivers. Each of this 8 is responsible for two tension sensors.
Those 8 transceivers are not read by the Raspberry Pi. Inbetween a STM32F103 acts as an SPI master to the transceivers and as an SPI slave to the Raspberry Pi and aggregates the individual tranceivers measurement for the Raspberry Pi.
The AD7730 provides a resolution of 24-bit and a two-staged filter setup. The setting of this filters and the resolution are hardcoded within the [firmware](https://github.com/arne48/load_cell_controller_firmware). 

#### Communication
1. __Between the controllers and the Raspberry Pi__
The communication between the Raspberry Pi and the controllers is realized using SPI with software chip-selects. The DAC of the air valve and the ADC of the pressure sensor controller are adressed using a 25MHz clock speed. 
The microcontroller of the load cell controller would support up to 18MHz, which according to the available clock dividers of the Raspberry Pi 3 leads to a usable SPI clock speed of 12.5 MHz.

2. __Between Raspberry Pi and external computer__
	
	The control of a specific muscle is provided in two flavors.
	
	__Activation Control__
	The command represents a normalized activation value between __-1 ... 1__ analogous to a fully open or fully blowing-off valve.
	
	__Pressure Control__
	The command represents the raw measurement of the ADC from __0 ... 65535__ analogous to a pressure from _0 to 1 MPa_.
	A common value for an empty muscle is about __3360__ with a variable maximum depending on the pressure currently available from the compressor.
	Once requested the desired pressure is maintained by a _PID_ controller which is available for additional tuning using __dynamic_reconfigure__ for each individual muscle controller.
	
	__Muscle Message__
	The current state of a muscle as a state message is also provided.
	The topic name is based on the following scheme:
	_/muscle\___\{ascending number\}__\_controller/state_
	
	Included by this message are the following details:
	- _name(String):_ Name of the described Muscle
	- _desired_pressure(Float64):_ If under pressure control this indictes the goal pressure
	- _current_pressure(Float64):_ The current pressure within the muscle
	- _tension(Float64):_ raw value from the tension sensor's load cell
	- _tension_filtered(Float64):_ filtered version of the tension's sensors values
	- _activation(Float64):_ current value the valve is currently actuated with
	- _control_mode(uint8):_ 0 indicates that muscle is under pressure control while 1 indicates activation control
	
	__MuscleCommand Message__
	This message is used within the _MusculatureCommand_ message and contains the information needed to update the current command of a muscle.
	
	Included by this message are the following details:
	-_name_ (String)
	-_pressure_ (Float64)
	-_activation_ (Float64)
	-_control_mode_ (enum CONTROL_MODE_BY_[PRESSURE or ACTIVATION])
		
	__MusculatureCommand Message__
	This message is part of the arl_hw_msgs package and is used to send a set of muscle commands in one message. 
	The topic name for these messages is:
	_/musculature/command_
	
	Included by this message are the following details:
	-_header_ (Header)
	-_muscle_commands_ (MuscleCommand[])
	
	__MusculatureState Message__
	This messag eis part of the arl_hw_msgs package and is used to publish the state of all muscles in one message. 
	The topic name for these messages is:
	_/musculature/state_
	
	Included by this message are the following details:
	-_header_ (Header)
	-_muscle_states_ (Muscle[])
	
	__Emergency Stop__
	To engage an _Emergency Stop_ a service with the service description _std_srv/Trigger_ can be called.
	The topic of the _Emergency Stop_ is:
	
	_/emergency_stop_
	
	This call blows-off the air of all muscles immediately.

#### Software Components
1. __Driver__
The driver is located in the [arl_hw](https://github.com/arne48/arl_hw) package.
It maintains the realtime control loop and orchestrates the reading and writing of the controllers as well as calls the individual muscle controllers.

2. __Custom Messages__
The available custom messages can be found in the [arl_hw_msgs](https://github.com/arne48/arl_hw_msgs) package
Currently the only one actively used is the _Muscle Message_ which is used for publishing the muscle's detailed state. Details of this message can be found under the earlier bullet point _Muscle Message_.

3. __Muscle Controller__
The custom _MuscleController_ which is located in the [arl_controllers](https://github.com/arne48/arl_controllers) package is in charge of publishing individual muscle states and also the PID controller which is used during pressure control and the filter for the raw tension measurements are part of this controller.

4. __Commons__
The [arl_commons](https://github.com/arne48/arl_commons) package contains test nodes and the launch file for starting the _diagnostic_aggregator_ for the driver.

5. __Muscle Tester rqt Plugin__
The [arl_muscle_tester](https://github.com/arne48/arl_muscle_tester) is a plugin for rqt which can be used to test the operability of individual muscles. A more detailed description can be found the it's [README](https://github.com/arne48/arl_muscle_tester/blob/master/README.md).

## Usage of the Package
In this section the start-up procedure of the robot and examples of it's usage will be presented.

### Starting the System
1. Power up the Robot and the Raspberry Pi
2. When the Raspberry Pi is booted up press the reset button on the controllers to bring them into a defined state.
3. Because the on the Raspberry Pi root access is needed for full GPIO access the driver needs to be started as follows:
```bash
sudo su
source /home/${USER}/.bashrc
roslaunch arl_hw arl_robot_driver.launch
``` 
4. Once the driver is started the power switch of the valves (left on the robots backside) can be activated.
5. Now the muscle controllers can be started with regular user permissions
```bash
roslaunch arl_controllers muscle_controller.launch
``` 
6. As the last step the _diagnostic_aggregator_ for the driver will get started by
```bash
roslaunch arl_commons diagnostic_aggregator.launch
``` 
7. Now it should be noticeable that all valves are moderately blowing-off air from the muscles.
8. The robot is ready to operate.

### Examples for using the System

#### Python
##### Controlling a Muscle
```python
#!/usr/bin/env python
import rospy
from arl_hw_msgs.msg import MuscleCommand, MusculatureCommand

def commander():
    rospy.init_node('muscle_commander', anonymous=True)
    muscle_cmd_pub = rospy.Publisher('/musculature/command', MusculatureCommand, queue_size=10)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        
        muscle_command = MuscleCommand()
        muscle_command.name = 'muscle_1'
        muscle_command.pressure = 8000
        muscle_command.activation = 0
        muscle_command.control_mode = 0
        musculature_command.muscle_commands.append(muscle_command)
        
        muscle_cmd_pub.publish(musculature_command)
        rate.sleep()
        
        musculature_command.muscle_commands[0].pressure = 4000
        muscle_cmd_pub.publish(musculature_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
```


##### Receiving Muscle Details
```python
#!/usr/bin/env python
import rospy
from arl_hw_msgs.msg import Muscle, MusculatureState

def musculature_state_callback(msg):
    for muscle in msg.muscle_states:
        rospy.loginfo("My muscle %s flexed with an activation of %f", muscle.name, muscle.activation)
    
def observer():
    rospy.init_node('muscle_observer', anonymous=True)
    self._musculature_state_subscriber = rospy.Subscriber('/musculature/state', MusculatureState, musculature_state_callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    observer()
```

## Important Remarks
### Hardware
#### Raspberry Pi 3 Setting of SPI Speed Divider
An initial idea was to set the speed of the SPI bus according to the max speed of the controller which would be either read or wrote to.
With this behavior it was thought to increase the overall cycle time by utilizing the individual controllers as efficient as possible.
Under normal conditions without extensive load on the Raspberry Pi this worked out fine.
But once the Raspberry Pi was put under more load it seems that the switching of the speed devision could take more time.
This lead to a behavior were the Raspberry Pi tried to change the SPI speed. But this took longer than the time till the SPI transfer was started. So the Pi started the transfer with one clock speed and changed it inbetween during the active transfer.
This lead to glitchy readings and writings to the controllers.
__Because of this behavior the SPI speed is now fixed to the maximum all used controllers can work with.__

#### Depreciation of Controller ID 0 and 1 on Platforms other than the Raspberry Pi 3
By using the bcm2835 library to utilize the Raspberry Pie's SPI and GPIO interfaces it is possible use the normally dedicated hardware chip selects as regular GPIOs when using the _BCM2835_SPI_CS_NONE_ setting.
This might not be possible on all platforms the driver will run on. It might be possible to set a related flag but the SPI block might hold control over the pins what makes them useless for software based addressing of the controllers.
Because of this and the PCB layout which hardwires these pins to solder bridges it should be common practice to stop using these IDs altogether to avoid incompatibilities when changing to a different platform.

### Software
#### Pitfalls related to the CMakeCache.txt when using options within the CMakeLists.txt
In newer versions the driver is supposed to be build with the usage of several options which can be set within the CMakeLists.txt
This should reduce the dependencies when building the driver on a specific platform or on a PC for testing or development reasons.
This is a common practice when using CMake. Something counterintuitive is how and when these options are evaluated.
Since they should introduce an additional possibility to configure your build it would make sense that this is done on every build.
__Unfortunately from my experience this is done only once when the project is build the first time. Then the options are cached in the CMakeCache.txt file within the _build_ folder of your catkin workspace. _catkin_make clean_ does not solve this issue. So if you change any option the most safe way to make sure the changes are applied is to delete the whole content of the _build_ folder__