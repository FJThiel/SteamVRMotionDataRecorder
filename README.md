# SteamVR Motion Data Recorder
A simple command line application to record motion data from the SteamVR API

## Licencing
This piece of software is published under the MIT Licence unless marked otherwise. Note that the submodules in the folder thirdParty are subject to their respective licences:
- GLM (Happy Bunny and MIT Licence)
- OpenVR (3-Clause BSD Licence)

## About This Application
This simple command line application was developed at the University College London to interface with Valve's OpenVR API and record motion data and button inputs by the VR user. This data is recorded on the hard drive during execution in CSV files.
Additional provisions have been made to enter a participant ID and to automatically detect the game that is played. Both is included in the file name.

## Building The Recording Application

#### Requirements
* Visual Studio (I used VS 2017) 
* CMake

#### Known Working Configurations
| Visual Studio | CMake  | OpenVR | glm     |
| :-----------: | :----: | :----: | :-----: |
| 2017 (15.9.48)| 3.21.4 | 1.16.8 | cc98465 |


#### Preparation
1. Clone repository

2. Initialise the submodules  
<code>git submodule init</code>  
<code>git submodule update</code>

3. Open CMake and load the CMake file from the root directory of the repository.

4. Set the build directory to  
   <code>\<repo>\build</code>

5. Press "Configure" in Cmake. You may need to select the compiler you want. In my case Visual Studio 15 (2017)

6. After a successful configuration, press "Generate"

7. Open SteamVRMotionDataRecorder.sln in the folder  
<code>\<repo\>\build</code>

## Installing the Application

You can store the application wherever you want on your computer. However, I would advise against network shares or thumb drives. Make sure that all files that are in the folder stay together.

## Using the Application

1. Start SteamVR.

2. Start game that you want to record (optional).

3. Start the recorder application via the command line.

4. Enter the participant ID and press enter.

5. If a game was detected, the application will let you know now. If no game is detected, a generic designation is used for the file.

6. When you are ready to start the recording, press the space bar. The application will count down via audio cues and then start at the end of the last tone.

7. When you want to stop the recording, press the space bar again.

8. The application will offer you to do another recording. You can either type "N" to close the application or type "Y" to continue. In the latter case, you will return to step 5.