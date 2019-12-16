#include "pch.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <time.h>
#include <numeric>
#include<fstream>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
	}
	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;
	}
	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		myo->unlock(myo::Myo::unlockHold);
		if (reference_quat_saved == true) {
			actual_quat[0] = quat.w();
			actual_quat[1] = quat.x();
			actual_quat[2] = quat.y();
			actual_quat[3] = quat.z();

			new_quat = true;
		} 
		else {
			myo::Quaternion<float> conjugate_quat = quat.conjugate();

			reference_quat[0] = conjugate_quat.w();
			reference_quat[1] = conjugate_quat.x();
			reference_quat[2] = conjugate_quat.y();
			reference_quat[3] = conjugate_quat.z();

			reference_quat_saved = true;
		}
	}

	void quaternion_product(float(&q)[4], float(&r)[4], float(&result)[4])
	{
		result[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
		result[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
		result[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
		result[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
	}

	bool quat_to_rpy() {
		if (new_quat == true) {

			quaternion_product(actual_quat, reference_quat, rotated_quat);

			// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
			float roll = std::atan2(2.0f * (rotated_quat[0] * rotated_quat[1] + rotated_quat[2] * rotated_quat[3]),
				1.0f - 2.0f * (rotated_quat[1] * rotated_quat[1] + rotated_quat[2] * rotated_quat[2]));
			float pitch = std::asin(std::max(-1.0f, std::min(1.0f, 2.0f * (rotated_quat[0] * rotated_quat[2] - rotated_quat[3] * rotated_quat[1]))));
			float yaw = std::atan2(2.0f * (rotated_quat[0] * rotated_quat[3] + rotated_quat[1] * rotated_quat[2]),
				1.0f - 2.0f * (rotated_quat[2] * rotated_quat[2] + rotated_quat[3] * rotated_quat[3]));
			// Convert the floating point angles in radians to a scale from 0 to 18.

			roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 1.0f) * 18) - 18;
			pitch_w = static_cast<int>((pitch + (float)M_PI) / (M_PI * 1.0f) * 18) - 18;
			yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 1.0f) * 18) - 18;

			if (roll_w < 2 && roll_w > -2) roll_w = 0;
			if (pitch_w < 2 && pitch_w > -2) pitch_w = 0;
			if (yaw_w < 2 && yaw_w > -2) yaw_w = 0;

			new_quat = false;			
			return true;
		}
		else return false;
		
	}
	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo * myo, uint64_t timestamp, myo::Pose pose)
	{
    		currentPose = pose;
    
    		if (pose != myo::Pose::unknown) {
        		myo->notifyUserAction();
    		}
	}
	// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}
	// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
	// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
	// when Myo is moved around on the arm.
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
	{
		onArm = false;
	}
	// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	void onUnlock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = true;
	}
	// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	void onLock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = false;
	}
	// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
	// For this example, the functions overridden above are sufficient.
	// We define this function to print the current values that were updated by the on...() functions above.

	// These values are set by onArmSync() and onArmUnsync() above.
	bool onArm;
	myo::Arm whichArm;
	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked;
	// These values are set by onOrientationData() and onPose() above.
	int roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;

	bool reference_quat_saved = false;
	float reference_quat[4];

	float actual_quat[4];
	bool new_quat = false;

	float rotated_quat[4];
};


int main(array<System::String^>^ args) //args is a pointer to an array of System::String pointers

{
	//We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.19gr366");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		System::IO::Ports::SerialPort port;

		port.PortName = "COM7";
		port.BaudRate = 57600;
		port.WriteTimeout = 100;
		port.ReadTimeout = 500;

		port.Open();

		if (!port.IsOpen) {
			throw std::runtime_error("Unable to established serial connection.");
		}

		std::cout << "Serial connection has been established." << std::endl << std::endl;

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		std::string oldPoseString = "unknown";
		clock_t old_time, present_time;

		old_time = clock();

		int counter = 0;

		// Finally we enter our main loop.
		while (true) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			hub.run(1000 / 20);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			int hertz = 20;

			present_time = clock();
			double time = double(present_time - old_time) / double(CLOCKS_PER_SEC);

			if (time > (1 / hertz)) {
				if (collector.quat_to_rpy()) {
					std::string message;
					message.append("$");
					if (collector.currentPose.toString() == "waveIn") message.append("-3");
					else if (collector.currentPose.toString() == "waveOut") message.append("3");
					else message.append("0");
					message.append(",");
					message.append(std::to_string(collector.pitch_w));
					message.append(",");
					message.append(std::to_string(collector.yaw_w));
					message.append(",");
					if (collector.currentPose.toString() == "fist") message.append("1");
					else if (collector.currentPose.toString() == "fingersSpread") message.append("2");
					else message.append("0");
					message.append("#");
					std::cout << message << std::endl;
					System::String^ output = gcnew System::String(message.c_str());
					port.Write(output);

					old_time = present_time;
				}
			}

		}
	}
		// If a standard exception occurred, we print out its message and exit.
	
	catch (const std::exception & e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}

	return 0;
}

