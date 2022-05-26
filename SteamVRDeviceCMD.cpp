// SteamVRDeviceCMD.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

//#include <openvr.h>
#include "thirdParty\openvr\headers\openvr.h"
#include <chrono>

#include <fstream>

#include <Windows.h>

#include <filesystem>

#include "thirdParty/glm/glm/glm.hpp"
#include "thirdParty/glm/glm/matrix.hpp"
#include "thirdParty/glm/glm/gtc/matrix_transform.hpp"
#include "thirdParty/glm/glm/gtc/quaternion.hpp"
#include "thirdParty/glm/glm/gtx/quaternion.hpp"

// how many samples per second should be taken
const int samplingFrequency = 10;

float samplingInterval = 1000 / samplingFrequency;


// Action handles: Main
vr::VRActionHandle_t m_handleSetMain;

// Action handles: Left
vr::VRActionHandle_t m_handleControllerLeftButtonTrigger;
vr::VRActionHandle_t m_handleControllerLeftButtonGrip;
vr::VRActionHandle_t m_handleControllerLeftButtonTouchpad;
vr::VRActionHandle_t m_handleControllerLeftButtonJoystick;
vr::VRActionHandle_t m_handleControllerLeftButtonButtonA;
vr::VRActionHandle_t m_handleControllerLeftButtonButtonB;
vr::VRActionHandle_t m_handleControllerLeftButtonButtonSystem;
vr::VRActionHandle_t m_handleControllerLeftButtonButtonMenu;
vr::VRActionHandle_t m_handleControllerLeftAxisTrigger;
vr::VRActionHandle_t m_handleControllerLeftAxisGrip;
vr::VRActionHandle_t m_handleControllerLeftAxisJoystick;
vr::VRActionHandle_t m_handleControllerLeftAxisTouchpad;
vr::VRActionHandle_t m_handleControllerLeftTouchedTrigger;
vr::VRActionHandle_t m_handleControllerLeftTouchedGrip;
vr::VRActionHandle_t m_handleControllerLeftTouchedTouchpad;
vr::VRActionHandle_t m_handleControllerLeftTouchedJoystick;
vr::VRActionHandle_t m_handleControllerLeftTouchedButtonA;
vr::VRActionHandle_t m_handleControllerLeftTouchedButtonB;
vr::VRActionHandle_t m_handleControllerLeftTouchedButtonSystem;
vr::VRActionHandle_t m_handleControllerLeftTouchedButtonMenu;
vr::VRActionHandle_t m_handleControllerLeftSqueezedGrip;

// Action handles: Right
vr::VRActionHandle_t m_handleControllerRightButtonTrigger;
vr::VRActionHandle_t m_handleControllerRightButtonGrip;
vr::VRActionHandle_t m_handleControllerRightButtonTouchpad;
vr::VRActionHandle_t m_handleControllerRightButtonJoystick;
vr::VRActionHandle_t m_handleControllerRightButtonButtonA;
vr::VRActionHandle_t m_handleControllerRightButtonButtonB;
vr::VRActionHandle_t m_handleControllerRightButtonButtonSystem;
vr::VRActionHandle_t m_handleControllerRightButtonButtonMenu;
vr::VRActionHandle_t m_handleControllerRightAxisTrigger;
vr::VRActionHandle_t m_handleControllerRightAxisGrip;
vr::VRActionHandle_t m_handleControllerRightAxisJoystick;
vr::VRActionHandle_t m_handleControllerRightAxisTouchpad;
vr::VRActionHandle_t m_handleControllerRightTouchedTrigger;
vr::VRActionHandle_t m_handleControllerRightTouchedGrip;
vr::VRActionHandle_t m_handleControllerRightTouchedTouchpad;
vr::VRActionHandle_t m_handleControllerRightTouchedJoystick;
vr::VRActionHandle_t m_handleControllerRightTouchedButtonA;
vr::VRActionHandle_t m_handleControllerRightTouchedButtonB;
vr::VRActionHandle_t m_handleControllerRightTouchedButtonSystem;
vr::VRActionHandle_t m_handleControllerRightTouchedButtonMenu;
vr::VRActionHandle_t m_handleControllerRightSqueezedGrip;


void GiveCountdown(int seconds)
{
	if (seconds == 0)
		return;

	for (int i = 0; i < seconds; i++) {
		Beep(500, 500);
		Sleep(500);
	}

	Beep(750, 1000);
}

std::string GetStringProperty(vr::TrackedDeviceIndex_t deviceIndex, vr::ETrackedDeviceProperty deviceProperty)
{
	vr::ETrackedPropertyError pError;
	char propertyChar[1028] = { '\0' };
	vr::VRSystem()->GetStringTrackedDeviceProperty(deviceIndex, deviceProperty, propertyChar, 1028, &pError);

	return std::string(propertyChar);
}

// From:
// https://www.codeproject.com/Articles/1171122/How-to-Get-Raw-Positional-Data-from-HTC-Vive
vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

bool GetDigitalActionData(vr::VRActionHandle_t handle) {
	vr::InputDigitalActionData_t actionData;
	vr::VRInput()->GetDigitalActionData(handle, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
	return actionData.bActive && actionData.bState;
}

float GetAnalogueActionData1D(vr::VRActionHandle_t handle) {
	vr::InputAnalogActionData_t actionData;
	vr::VRInput()->GetAnalogActionData(handle, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
	return actionData.x;
}

glm::vec2 GetAnalogueActionData2D(vr::VRActionHandle_t handle) {
	vr::InputAnalogActionData_t actionData;
	vr::VRInput()->GetAnalogActionData(handle, &actionData, sizeof(actionData), vr::k_ulInvalidInputValueHandle);
	return glm::vec2(actionData.x, actionData.y);
}

bool InitOpenVR() {
	vr::HmdError eLastHMDError = vr::VRInitError_None;
	vr::IVRSystem *pVRSystem = vr::VR_Init(&eLastHMDError, vr::VRApplication_Other);
	//vr::IVRSystem *pVRSystem = vr::VR_Init(&eLastHMDError, vr::EVRApplicationType::VRApplication_Scene);


	if (eLastHMDError != vr::VRInitError_None)
	{		
		return false;
	}
	else
	{
		return true;
	}
}


bool InitManifest() {
	std::string curPath = std::experimental::filesystem::v1::current_path().string();
	std::string manifestPath = curPath.append("\\Bindings\\actions.json");
	vr::EVRInputError error = vr::VRInput()->SetActionManifestPath(manifestPath.c_str());

	if (error != vr::VRInitError_None) {		
		return false;
	}
	else {
		return true;
	}
}

void RetreiveActionHandles() {
	vr::VRInput()->GetActionHandle("/actions/main", &m_handleSetMain);

	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonTrigger", &m_handleControllerLeftButtonTrigger);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonGrip", &m_handleControllerLeftButtonGrip);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonTouchpad", &m_handleControllerLeftButtonTouchpad);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonJoystick", &m_handleControllerLeftButtonJoystick);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonButtonA", &m_handleControllerLeftButtonButtonA);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonButtonB", &m_handleControllerLeftButtonButtonB);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonButtonSystem", &m_handleControllerLeftButtonButtonSystem);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftButtonButtonMenu", &m_handleControllerLeftButtonButtonMenu);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftAxisTrigger", &m_handleControllerLeftAxisTrigger);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftAxisGrip", &m_handleControllerLeftAxisGrip);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftAxisJoystick", &m_handleControllerLeftAxisJoystick);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftAxisTouchpad", &m_handleControllerLeftAxisTouchpad);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedTrigger", &m_handleControllerLeftTouchedTrigger);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedGrip", &m_handleControllerLeftTouchedGrip);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedTouchpad", &m_handleControllerLeftTouchedTouchpad);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedJoystick", &m_handleControllerLeftTouchedJoystick);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedButtonA", &m_handleControllerLeftTouchedButtonA);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedButtonB", &m_handleControllerLeftTouchedButtonB);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedButtonSystem", &m_handleControllerLeftTouchedButtonSystem);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftTouchedButtonMenu", &m_handleControllerLeftTouchedButtonMenu);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerLeftSqueezedGrip", &m_handleControllerLeftSqueezedGrip);

	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonTrigger", &m_handleControllerRightButtonTrigger);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonGrip", &m_handleControllerRightButtonGrip);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonTouchpad", &m_handleControllerRightButtonTouchpad);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonJoystick", &m_handleControllerRightButtonJoystick);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonButtonA", &m_handleControllerRightButtonButtonA);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonButtonB", &m_handleControllerRightButtonButtonB);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonButtonSystem", &m_handleControllerRightButtonButtonSystem);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightButtonButtonMenu", &m_handleControllerRightButtonButtonMenu);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightAxisTrigger", &m_handleControllerRightAxisTrigger);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightAxisGrip", &m_handleControllerRightAxisGrip);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightAxisJoystick", &m_handleControllerRightAxisJoystick);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightAxisTouchpad", &m_handleControllerRightAxisTouchpad);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedTrigger", &m_handleControllerRightTouchedTrigger);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedGrip", &m_handleControllerRightTouchedGrip);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedTouchpad", &m_handleControllerRightTouchedTouchpad);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedJoystick", &m_handleControllerRightTouchedJoystick);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedButtonA", &m_handleControllerRightTouchedButtonA);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedButtonB", &m_handleControllerRightTouchedButtonB);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedButtonSystem", &m_handleControllerRightTouchedButtonSystem);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightTouchedButtonMenu", &m_handleControllerRightTouchedButtonMenu);
	vr::VRInput()->GetActionHandle("/actions/main/in/ControllerRightSqueezedGrip", &m_handleControllerRightSqueezedGrip);
}

void StartRecording(std::string participantKey, std::string gameName) {
	
	// Assemble file name
	auto time = std::chrono::system_clock::now();
	long long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();

	std::string filenameMotion = "motion_" + participantKey + "_" + gameName + "_" + std::to_string(timestamp) + ".csv";
	std::string filenameInput = "input_" + participantKey + "_" + gameName + "_" + std::to_string(timestamp) + ".csv";

	
	
	// Open the file
	std::ofstream motionDataStream;
	std::ofstream inputDataStream;

	motionDataStream.open(filenameMotion);
	inputDataStream.open(filenameInput);
	if (!motionDataStream || !inputDataStream) {
		std::cout << "Could not open files\n";
		return;
	}

	// Print header for CSV
	motionDataStream << "timestamp,deviceIndex,controllerType,serial,position.x,position.y,position.z,rotation.w,rotation.x,rotation.y,rotation.z,v.x,v.y,v.z,av.x,av.y,av.z" << std::endl;
	inputDataStream << "timestamp, Side, ButtonTrigger, ButtonGrip, ButtonTouchpad, ButtonJoystick, ButtonButtonA, ButtonButtonB, ButtonButtonSystem, ButtonButtonMenu, AxisTrigger, AxisGrip, AxisGripForce, AxisJoystickX, AxisJoystickY, AxisTouchpadX, AxisTouchpadY, TouchedTrigger, TouchedGrip, TouchedTouchpad, TouchedJoystick, TouchedButtonA, TouchedButtonB, TouchedButtonSystem, TouchedButtonMenu" << std::endl;

	std::cout << "All ready. Press Space to start the recording" << std::endl;

	bool recording = false;

	// endless loop waiting for the command to start
	while (!recording) {
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
			recording = true;
		}
	}

	std::cout << "Starting recording in three seconds. Get Ready!" << std::endl;

	GiveCountdown(3);

	std::cout << "Recording. Press Space to stop." << std::endl;




	// storage for device starting positions
	glm::vec3 deviceStartingPos[vr::k_unMaxTrackedDeviceCount];
	glm::quat deviceStartingRot[vr::k_unMaxTrackedDeviceCount];

	bool startingValuesCollectedPos[vr::k_unMaxTrackedDeviceCount] = { 0 };
	bool startingValuesCollectedRot[vr::k_unMaxTrackedDeviceCount] = { 0 };



	// Endless loop for the recording
	while (recording)
	{
		// check if space is pressed
		if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
			recording = false;
		}


		//std::cout << "Querrying devices...\n\n";


		auto time = std::chrono::system_clock::now();

		long long timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();

		//std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count() << "\n";


		// timing stuff
		float fSecondsSinceLastVsync;
		vr::VRSystem()->GetTimeSinceLastVsync(&fSecondsSinceLastVsync, NULL);
		float fDisplayFrequency = vr::VRSystem()->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
		float fFrameDuration = 1.f / fDisplayFrequency;
		float fVsyncToPhotons = vr::VRSystem()->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);
		float fPredictedSecondsFromNow = fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;

		// Querry all poses
		vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, fPredictedSecondsFromNow, devicePoses, vr::k_unMaxTrackedDeviceCount);



		for (uint32_t deviceIndex = 0; deviceIndex < vr::k_unMaxTrackedDeviceCount; deviceIndex++) {

			//std::cout << "Device" << deviceIndex << "\n";

			if (!vr::VRSystem()->IsTrackedDeviceConnected(deviceIndex)) {
				//std::cout << "Not connected\n\n";
				continue;
			}

			std::string controllerType = GetStringProperty(deviceIndex, vr::ETrackedDeviceProperty::Prop_ControllerType_String);
			//std::cout << "Type: " << controllerType << "\n";

			std::string serial = GetStringProperty(deviceIndex, vr::ETrackedDeviceProperty::Prop_SerialNumber_String);
			//std::cout << "Serial: " << serial << "\n";


			vr::TrackedDevicePose_t* pose = devicePoses + deviceIndex;
			vr::HmdMatrix34_t* poseMat = &(pose->mDeviceToAbsoluteTracking);
			if (pose->bPoseIsValid && pose->bDeviceIsConnected) {
				glm::vec3 position = glm::vec3(poseMat->m[0][3], poseMat->m[1][3], poseMat->m[2][3]);


				//std::cout << "Position: " << position.x << " | " << position.y << " | " << position.z << "\n";

				// print rotation matrix (row by row)
				//std::cout << "Rotation Matrix (row major)\n";
				//std::cout << poseMat->m[0][0] << " | " << poseMat->m[0][1] << " | " << poseMat->m[0][2] << "\n";
				//std::cout << poseMat->m[1][0] << " | " << poseMat->m[1][1] << " | " << poseMat->m[1][2] << "\n";
				//std::cout << poseMat->m[2][0] << " | " << poseMat->m[2][1] << " | " << poseMat->m[2][2] << "\n";

				//std::cout << "\n\n";


				// If we do not have starting values yet for position, collect them
				if (!startingValuesCollectedPos[deviceIndex]) {
					deviceStartingPos[deviceIndex] = position;
					startingValuesCollectedPos[deviceIndex] = true;
				}

				// subtract starting value from current position
				//position = position - deviceStartingPos[deviceIndex];

				// get rotation and convert it to quaternioin
				vr::HmdQuaternion_t rotationRaw = GetRotation(*poseMat);
				glm::quat rotation = glm::quat(rotationRaw.w, rotationRaw.x, rotationRaw.y, rotationRaw.z);

				// If we do not have starting values yet for rotation, collect them
				if (!startingValuesCollectedRot[deviceIndex]) {
					deviceStartingRot[deviceIndex] = rotation;
					startingValuesCollectedRot[deviceIndex] = true;
				}

				// "subtract" starting rotation from current rotation
				//rotation = rotation * glm::inverse(deviceStartingRot[deviceIndex]);

				// get angular velocities
				auto angularVelocityRaw = pose->vAngularVelocity.v;
				glm::vec3 angularVelocity = glm::vec3(angularVelocityRaw[0], angularVelocityRaw[1], angularVelocityRaw[2]);

				// get velocities
				auto velocityRaw = pose->vVelocity.v;
				glm::vec3 velocity = glm::vec3(velocityRaw[0], velocityRaw[1], velocityRaw[2]);

				// Assemble values and print line
				motionDataStream << timestamp << ","
					<< deviceIndex << ","
					<< controllerType << ","
					<< serial << ","
					<< position.x << ","
					<< position.y << ","
					<< position.z << ","
					<< rotation.w << ","
					<< rotation.x << ","
					<< rotation.y << ","
					<< rotation.z << ","
					<< velocity.x << ","
					<< velocity.y << ","
					<< velocity.z << ","
					<< angularVelocity.x << ","
					<< angularVelocity.y << ","
					<< angularVelocity.z << std::endl;

			}

		}


		vr::VRActiveActionSet_t actionSet = { 0 };
		actionSet.ulActionSet = m_handleSetMain;
		vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);



		// Buttons		
		bool controllerLeftButtonTrigger = GetDigitalActionData(m_handleControllerLeftButtonTrigger);
		bool controllerLeftButtonGrip = GetDigitalActionData(m_handleControllerLeftButtonGrip);
		bool controllerLeftButtonTouchpad = GetDigitalActionData(m_handleControllerLeftButtonTouchpad);
		bool controllerLeftButtonJoystick = GetDigitalActionData(m_handleControllerLeftButtonJoystick);
		bool controllerLeftButtonButtonA = GetDigitalActionData(m_handleControllerLeftButtonButtonA);
		bool controllerLeftButtonButtonB = GetDigitalActionData(m_handleControllerLeftButtonButtonB);
		bool controllerLeftButtonButtonSystem = GetDigitalActionData(m_handleControllerLeftButtonButtonSystem);
		bool controllerLeftButtonButtonMenu = GetDigitalActionData(m_handleControllerLeftButtonButtonMenu);

		bool controllerRightButtonTrigger = GetDigitalActionData(m_handleControllerRightButtonTrigger);
		bool controllerRightButtonGrip = GetDigitalActionData(m_handleControllerRightButtonGrip);
		bool controllerRightButtonTouchpad = GetDigitalActionData(m_handleControllerRightButtonTouchpad);
		bool controllerRightButtonJoystick = GetDigitalActionData(m_handleControllerRightButtonJoystick);
		bool controllerRightButtonButtonA = GetDigitalActionData(m_handleControllerRightButtonButtonA);
		bool controllerRightButtonButtonB = GetDigitalActionData(m_handleControllerRightButtonButtonB);
		bool controllerRightButtonButtonSystem = GetDigitalActionData(m_handleControllerRightButtonButtonSystem);
		bool controllerRightButtonButtonMenu = GetDigitalActionData(m_handleControllerRightButtonButtonMenu);


		// Axes
		float controllerLeftAxisTrigger = GetAnalogueActionData1D(m_handleControllerLeftAxisTrigger);
		float controllerLeftAxisGrip = GetAnalogueActionData1D(m_handleControllerLeftAxisGrip);
		float controllerLeftForceGrip = GetAnalogueActionData1D(m_handleControllerLeftSqueezedGrip);
		glm::vec2 controllerLeftAxisJoystick = GetAnalogueActionData2D(m_handleControllerLeftAxisJoystick);
		glm::vec2 controllerLeftAxisTouchpad = GetAnalogueActionData2D(m_handleControllerLeftAxisTouchpad);

		float controllerRightAxisTrigger = GetAnalogueActionData1D(m_handleControllerRightAxisTrigger);
		float controllerRightAxisGrip = GetAnalogueActionData1D(m_handleControllerRightAxisGrip);
		float controllerRightForceGrip = GetAnalogueActionData1D(m_handleControllerRightSqueezedGrip);
		glm::vec2 controllerRightAxisJoystick = GetAnalogueActionData2D(m_handleControllerRightAxisJoystick);
		glm::vec2 controllerRightAxisTouchpad = GetAnalogueActionData2D(m_handleControllerRightAxisTouchpad);


		// Touched
		bool controllerLeftTouchedTrigger = GetDigitalActionData(m_handleControllerLeftTouchedTrigger);
		bool controllerLeftTouchedGrip = GetDigitalActionData(m_handleControllerLeftTouchedGrip);
		bool controllerLeftTouchedTouchpad = GetDigitalActionData(m_handleControllerLeftTouchedTouchpad);
		bool controllerLeftTouchedJoystick = GetDigitalActionData(m_handleControllerLeftTouchedJoystick);
		bool controllerLeftTouchedButtonA = GetDigitalActionData(m_handleControllerLeftTouchedButtonA);
		bool controllerLeftTouchedButtonB = GetDigitalActionData(m_handleControllerLeftTouchedButtonB);
		bool controllerLeftTouchedButtonSystem = GetDigitalActionData(m_handleControllerLeftTouchedButtonSystem);
		bool controllerLeftTouchedButtonMenu = GetDigitalActionData(m_handleControllerLeftTouchedButtonMenu);

		bool controllerRightTouchedTrigger = GetDigitalActionData(m_handleControllerRightTouchedTrigger);
		bool controllerRightTouchedGrip = GetDigitalActionData(m_handleControllerRightTouchedGrip);
		bool controllerRightTouchedTouchpad = GetDigitalActionData(m_handleControllerRightTouchedTouchpad);
		bool controllerRightTouchedJoystick = GetDigitalActionData(m_handleControllerRightTouchedJoystick);
		bool controllerRightTouchedButtonA = GetDigitalActionData(m_handleControllerRightTouchedButtonA);
		bool controllerRightTouchedButtonB = GetDigitalActionData(m_handleControllerRightTouchedButtonB);
		bool controllerRightTouchedButtonSystem = GetDigitalActionData(m_handleControllerRightTouchedButtonSystem);
		bool controllerRightTouchedButtonMenu = GetDigitalActionData(m_handleControllerRightTouchedButtonMenu);


		//std::cout << "Button: Trigger: " << controllerLeftButtonTrigger << " | " << controllerRightButtonTrigger << std::endl;
		//std::cout << "Button: Grip: " << controllerLeftButtonGrip << " | " << controllerRightButtonGrip << std::endl;
		//std::cout << "Button: Touchpad: " << controllerLeftButtonTouchpad << " | " << controllerRightButtonTouchpad << std::endl;
		//std::cout << "Button: Joystick: " << controllerLeftButtonJoystick << " | " << controllerRightButtonJoystick << std::endl;
		//std::cout << "Button: Button A: " << controllerLeftButtonButtonA << " | " << controllerRightButtonButtonA << std::endl;
		//std::cout << "Button: Button B: " << controllerLeftButtonButtonB << " | " << controllerRightButtonButtonB << std::endl;
		//std::cout << "Button: Button System: " << controllerLeftButtonButtonSystem << " | " << controllerRightButtonButtonSystem << std::endl;
		//std::cout << "Button: Button Menu: " << controllerLeftButtonButtonMenu << " | " << controllerRightButtonButtonMenu << std::endl;

		//std::cout << "Axis: Trigger: " << controllerLeftAxisTrigger << " | " <<  controllerRightAxisTrigger << std::endl;
		//std::cout << "Axis: Grip: " << controllerLeftAxisGrip << " | " << controllerRightAxisGrip << std::endl;
		//std::cout << "Axis: Joystick: " << controllerLeftAxisJoystick.x << " | " << controllerLeftAxisJoystick.y << controllerRightAxisJoystick.x << " | " << controllerRightAxisJoystick.y << std::endl;
		//std::cout << "Axis: Touchpad: " << controllerLeftAxisTouchpad.x << " | " << controllerLeftAxisTouchpad.y << controllerRightAxisTouchpad.x << " | " << controllerRightAxisTouchpad.y << std::endl;

		//std::cout << "Touched: Trigger: " << controllerLeftTouchedTrigger << " | " << controllerRightTouchedTrigger << std::endl;
		//std::cout << "Touched: Grip: " << controllerLeftTouchedGrip << " | " << controllerRightTouchedGrip << std::endl;
		//std::cout << "Touched: Touchpad: " << controllerLeftTouchedTouchpad << " | " << controllerRightTouchedTouchpad << std::endl;
		//std::cout << "Touched: Joystick: " << controllerLeftTouchedJoystick << " | " << controllerRightTouchedJoystick << std::endl;
		//std::cout << "Touched: Button A: " << controllerLeftTouchedButtonA << " | " << controllerRightTouchedButtonA << std::endl;
		//std::cout << "Touched: Button B: " << controllerLeftTouchedButtonB << " | " << controllerRightTouchedButtonB << std::endl;
		//std::cout << "Touched: Button System: " << controllerLeftTouchedButtonSystem << " | " << controllerRightTouchedButtonSystem << std::endl;
		//std::cout << "Touched: Button Menu: " << controllerLeftTouchedButtonMenu << " | " << controllerRightTouchedButtonMenu << std::endl;


		inputDataStream << timestamp << ","
			<< "left" << ","
			<< controllerLeftButtonTrigger << ","
			<< controllerLeftButtonGrip << ","
			<< controllerLeftButtonTouchpad << ","
			<< controllerLeftButtonJoystick << ","
			<< controllerLeftButtonButtonA << ","
			<< controllerLeftButtonButtonB << ","
			<< controllerLeftButtonButtonSystem << ","
			<< controllerLeftButtonButtonMenu << ","
			<< controllerLeftAxisTrigger << ","
			<< controllerLeftAxisGrip << ","
			<< controllerLeftForceGrip << ","
			<< controllerLeftAxisJoystick.x << ","
			<< controllerLeftAxisJoystick.y << ","
			<< controllerLeftAxisTouchpad.x << ","
			<< controllerLeftAxisTouchpad.y << ","
			<< controllerLeftTouchedTrigger << ","
			<< controllerLeftTouchedGrip << ","
			<< controllerLeftTouchedTouchpad << ","
			<< controllerLeftTouchedJoystick << ","
			<< controllerLeftTouchedButtonA << ","
			<< controllerLeftTouchedButtonB << ","
			<< controllerLeftTouchedButtonSystem << ","
			<< controllerLeftTouchedButtonMenu << std::endl;

		inputDataStream << timestamp << ","
			<< "right" << ","
			<< controllerRightButtonTrigger << ","
			<< controllerRightButtonGrip << ","
			<< controllerRightButtonTouchpad << ","
			<< controllerRightButtonJoystick << ","
			<< controllerRightButtonButtonA << ","
			<< controllerRightButtonButtonB << ","
			<< controllerRightButtonButtonSystem << ","
			<< controllerRightButtonButtonMenu << ","
			<< controllerRightAxisTrigger << ","
			<< controllerRightAxisGrip << ","
			<< controllerRightForceGrip << ","
			<< controllerRightAxisJoystick.x << ","
			<< controllerRightAxisJoystick.y << ","
			<< controllerRightAxisTouchpad.x << ","
			<< controllerRightAxisTouchpad.y << ","
			<< controllerRightTouchedTrigger << ","
			<< controllerRightTouchedGrip << ","
			<< controllerRightTouchedTouchpad << ","
			<< controllerRightTouchedJoystick << ","
			<< controllerRightTouchedButtonA << ","
			<< controllerRightTouchedButtonB << ","
			<< controllerRightTouchedButtonSystem << ","
			<< controllerRightTouchedButtonMenu << std::endl;


		Sleep(samplingInterval);

	}

	motionDataStream.close();
	inputDataStream.close();

	std::cout << "Recording stopped." << std::endl;
}

std::string DetectGame() {
	// Query game name
	//std::cout << "Enter Game Name:" << std::endl;
	std::string gameName;
	//std::getline(std::cin, gameName);

	//std::cout << gameName << std::endl;

	auto id = vr::VRApplications()->GetCurrentSceneProcessId();
	vr::EVRApplicationError pError;
	char propertyChar[1028] = { '\0' };
	pError = vr::VRApplications()->GetApplicationKeyByProcessId(id, propertyChar, 1028);
	std::string applicationKeyString = std::string(propertyChar);

	// Check for known games
	//applicationKeyString.find()

	if (applicationKeyString.find("620980") != std::string::npos) {
		std::cout << "Game detected: BeatSaber" << std::endl;
		gameName = "BeatSaber";
	}
	else if (applicationKeyString.find("546560") != std::string::npos) {
		std::cout << "Game detected: HL-Alyx" << std::endl;
		gameName = "HL-Alyx";
	}
	else if (applicationKeyString.find("1079800") != std::string::npos) {
		std::cout << "Game detected: PistolWhip" << std::endl;
		gameName = "PistolWhip";
	}
	else if (applicationKeyString.find("448280") != std::string::npos) {
		std::cout << "Game detected: JobSimulator" << std::endl;
		gameName = "JobSimulator";
	}
	else if (applicationKeyString.find("960040") != std::string::npos) {
		std::cout << "Game detected: ClashOfChefs" << std::endl;
		gameName = "ClashOfChefs";
	}
	else if (applicationKeyString.find("418650") != std::string::npos) {
		std::cout << "Game detected: SpacePirateTrainer" << std::endl;
		gameName = "SpacePirateTrainer";
	}
	else if (applicationKeyString.find("450390") != std::string::npos) {
		std::cout << "Game detected: TheLab" << std::endl;
		gameName = "TheLab";
	}
	else if (applicationKeyString.empty()) {
		std::cout << "No game running." << std::endl;
		gameName = "None";
	}
	else {
		std::cout << "Game detected but not identified. Data will be logged under: " << applicationKeyString << std::endl;
		gameName = applicationKeyString;
	}

	return gameName;
}

int main()
{
	std::cout << "Hello World!\n";

	// Init OpenVR
	std::cout << "Initialising OpenVR\n";
	bool success = InitOpenVR();
	if (!success)
	{
		std::cout << "Error in the OpenVR initialisation process\n";
		return 0;
	}
	else
	{
		std::cout << "OpenVR Successfuly initialised\n";
	}


	// Init the action systems: Read manifest
	success = InitManifest();

	if (!success) {
		std::cout << "Error in the manifest initialisation" << std::endl;
		return 0;
	}
	else {
		std::cout << "Manifest read in" << std::endl;
	}

	// Init the action systems: Get action handles 
	RetreiveActionHandles();


	// Query user key
	std::cout << "Enter Participant Key:" << std::endl;
	std::string participantKey;
	std::getline(std::cin, participantKey);

	//std::cout << participantKey << std::endl;
	
	





	bool running = true;

	do {

		std::string gameName = DetectGame();

		StartRecording(participantKey, gameName);


		std::cout << "Do you want to do another recording? (Y/N)" << std::endl;
		std::string response;
		std::getline(std::cin >> std::ws, response);

		if (response.compare("y") != 0 && response.compare("Y") != 0) {
			running = false;
		}

	} while (running);
	
}




