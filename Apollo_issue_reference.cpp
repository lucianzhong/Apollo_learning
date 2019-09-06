

1. 

	RTK Replay planning mode with additional planning tasks and driving scenarios
	It's it possible to configure and run planning module in RTK Replay mode with ability to change line or emergency stop to avoid obstacles collision?
	No, you need use standard mode for those functions.



2.

	Hello,

	The following code confuses me, anyone can explain its grammar？

	[]() -> LocalizationBase* { return new RTKLocalization(); });

	[]()-> means what?
	It is in Localization.cc of apollo1.0 and line 38.

	thanks.

	 @krishnatoshniwal
	 
	krishnatoshniwal commented on May 6
	Its a lambda function with no input parameter and the return object having the type LocalizationBase*. Look up on lambda expressions here
	https://en.cppreference.com/w/cpp/language/lambda



3.

		Steps to reproduce the issue:
		Please use bullet points and include as much details as possible:
		bash ./docker/scripts/dev_start.sh
		bash ./docker/scripts/dev_into.sh
		bash ./apollo.sh build
		bash scripts/bootstrap.sh
		Tasks: Sim Control: ON
		Module Controller: Routing: ON
		Module Controller: Planning: ON
		Default Routing Sunnyvale Loop: ON
		Gives me the screen shot below. Planning automatically OFF.

			

4.
  // canbus component does not exit gracefully #8053

Describe the bug
As subject

To Reproduce
Steps to reproduce the behavior:

Launch canbus component using mainboard
>>mainboard -d /apollo/modules/canbus/dag/canbus.dag
Shutdown it using Ctrl+C
The console shows:
terminate called without an active exception
Abort 
Cause
No cleanup of those handles: vehicle_controller_, can_receiver_, can_sender_, and can_client_.

Fix
The cyber component has a cleanup hook function called Clear() which will be called inside ComponentBase::Shutdown(). Override of this Clear() function inside CanbusComponent fixes the issue.

void CanbusComponent::Clear() {

  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();

  AINFO << "Cleanup Canbus component";
}



5. 

There are three relative map mode in apollo 2.5:
Mode 1: perception
Mode 2: navigator + perception
Mode 3: HDMap + navigator + perception.

Does apollo 3.5 also support these modes?

Those modes are not supported in 3.5 yet.



6. // How can I test the control module without a real car?


Apollo version:3.5
I have been trying to test the performance of the sample vehicle lincoln mx8 under a given complex path. After I compiled control_component_test.cc in apollo/modules/control and manually input a straight path which started at (0,0,0) and ended at (4.6,0,0), I found the .INFO file like this:

Log file created at: 2019/03/25 13:34:28
Running on machine: in_dev_docker
Log line format: [IWEF]mmdd hh:mm:ss.uuuuuu threadid file:line] msg
I0325 13:34:28.812129 11484 control_component.cc:42] Control init, starting ...
I0325 13:34:28.837960 11484 control_component.cc:48] Conf file: /apollo/modules/control/testdata/conf/control_conf.pb.txt is loaded.
I0325 13:34:28.837970 11484 control_component.cc:50] Conf file: is loaded.
I0325 13:34:28.837975 11484 controller_agent.cc:36] Only support MPC controller or Lat + Lon controllers as of now
I0325 13:34:28.838163 11484 lat_controller.cc:84] Using LQR-based Lateral Controller
I0325 13:34:28.838241 11484 lon_controller.cc:76] LON_CONTROLLER used.
I0325 13:34:28.838587 11484 lat_controller.cc:250] Lateral control gain scheduler loaded
I0325 13:34:28.838889 11484 lat_controller.cc:155] LQR-based Lateral Controller begin.
I0325 13:34:28.838897 11484 lat_controller.cc:156] [LatController parameters] mass_: 2080, iz_: 4208.3, lf_: 1.4224, lr_: 1.4224
I0325 13:34:28.838944 11484 controller_agent.cc:94] Controller init done!
I0325 13:34:28.838956 11484 lon_controller.cc:128] Control calibration table loaded
I0325 13:34:28.838973 11484 lon_controller.cc:129] Control calibration table size is 2694
I0325 13:34:28.840996 11484 controller_agent.cc:94] Controller <LON_CONTROLLER> init done!
I0325 13:34:28.850347 11484 control_component.cc:98] Control resetting vehicle state, sleeping for 1000 ms ...
I0325 13:34:29.850587 11484 control_component.cc:104] Control default driving action is STOP
E0325 13:34:29.852226 11491 control_component.cc:265] Chassis msg is not ready!
I0325 13:34:29.868499 11495 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.874752 11493 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.885205 11491 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.896140 11501 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.907552 11500 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.917747 11486 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
...

Now I'm wondering if I have missed some important configuration, or it's just unable to test the control module without a real car (with real GPS message input)?


Not as of now, but stay tuned ! :)




7. 