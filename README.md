# gulliview_2025

GulliView
=============

Locate AprilTags using the legendary GulliView!

The latest version of mobility model for gulliview with multithreading.

Authors
=============

Original author: Edwin Olson <ebolson@umich.edu>  
C++ port and modifications: Matt Zucker <mzucker1@swarthmore.edu>

***

Code has been modified for Vision Based Localization of Autonomous
Vehicles in the Gulliver Project at Chalmers University, Sweden

Modification Authors:  
Bayson Xie <beichen@chalmers.se>
Emil Nylander <emilnyla@chalmers.se>
Elias Svensson <eliasve@chalmers.se>


***
Code has been modified in the 2025 B.sc. project
at Chalmers University of Technology,
code has been split into files, pixel undistorsion and global coordination 
has been implemented as well as the GulliView logs program has been created

Modification Authors: 
Emil Nylander <emilnyla@chalmers.se>
Elias Svensson <eliasve@chalmers.se>

***

Helper files and other functions have also been merged with the original
project to help with certain functionalities:

Copyright (c) 2010  Chen Feng (cforrest (at) umich.edu)

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

More information can be found at: https://code.google.com/p/cv2cg/

***

Code has been further modified in "DAT295/DIT669 Autonomous and cooperative
vehicular systems" at Chalmers University of Technology, Sweden. Now depends
on the original AprilTag library instead of being a fork of the C++ AprilTag
port, achieving performance gains.

Modification Authors:  
Bayson Xie <beichen@chalmers.se>

***

Code has been modified in the 2024 B.sc. project
at Chalmers University of Technology,
Improves performance by implementing multithreading
as well as updating the transformation function for image undistortion

The Membership_service has been integrated, and several improvements have been made. 
Additionally, a new script, Transmitter.cpp, has been created. 
This script utilizes the shared memory (IPC) mechanism to broadcast a universal message.

Modification Authors: 
Johannes Johansson <johuc@chalmers.se>
Keivan Habibi <keivanh@chalmers.se>

Ali Shirzad <alishir@chalmers.se>
Ali Soltani <aliso@chalmers.se>

***

Requirements
============

For the following two parts (requirements and building, feel free to skip them if you are in the lab room. )

Tested on Debian 12 and Ubuntu. Make sure that dependencies are installed:

    sudo apt update && sudo apt install cmake libopencv-dev libboost-all-dev

We also need to install AprilTag:

    git clone https://github.com/AprilRobotics/apriltag.git
    cd apriltag
    cmake .
    sudo make install

Building
========

To compile the code, 

    git clone https://beichen1@bitbucket.org/automationarticulatedvehicles/gulliview_multithreading.git
    cd gulliview_multithreading
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

Running GulliView
=================

The binary is located in the build directory. Command line
options can be seen by running `./GulliView -h`. For running GulliView in
the eg5355 lab on all cameras, there are the scripts `startCamerasBroadcast1080.sh`
for Full HD resloution and `startCamerasBroadcast2160.sh` for 4K resolution
in the build directory.

----------
To run membership_service:

1- Run the script "startGulliViewMultithread.sh" on the GulliView machine. (Previous multiprocessing mode: "startGulliViewMembership.sh")
2- Run command "roscore" in one terminal and open the other one to run the script "robot_heartbeat_node.py" from the repository (command "python3 robot_heartbeat_node.py")
 "roswifibot-management-2023" on the wifibots computer. (address "/home/wifibot/roswifibot-management-2023/src/robot_heartbeat/src")  

Controlling Vehicle
================= 

# First, connect to wifibot3, and open two different terminals with successful connection

rlaunch # starts the drivers, in the first terminal

rkeyboard # starts the keyboard controller, in the second terminal

rqt # control the robot manually

It's not working :(
=================

* Has the right tag family been provided using the `-t` flag?
* Make sure the user running the program is in the video group:  
  `sudo useradd <user> video`
* If GulliView is not running with the `-n` (nogui) flag, X forwarding 
needs to be enabled if connected via SSH.
* The camera IDs presented by the OS may have changed. Try changing the 
command line option for the camera device (`-d <device>`).
* You may have trouble if multiple cameras are connected via the same USB hub.
* Using a powered USB hub instead of the on-board ports may work sometimes.

**Edit a file, create a new file, and clone from Bitbucket in under 2 minutes**

When you're done, you can delete the content in this README and update the file with details for others getting started with your repository.

*We recommend that you open this README in another tab as you perform the tasks below. You can [watch our video](https://youtu.be/0ocf7u76WSo) for a full demo of all the steps in this tutorial. Open the video in a new tab to avoid leaving Bitbucket.*

---

## Edit a file

You’ll start by editing this README file to learn how to edit a file in Bitbucket.

1. Click **Source** on the left side.
2. Click the README.md link from the list of files.
3. Click the **Edit** button.
4. Delete the following text: *Delete this line to make a change to the README from Bitbucket.*
5. After making your change, click **Commit** and then **Commit** again in the dialog. The commit page will open and you’ll see the change you just made.
6. Go back to the **Source** page.

---

## Create a file

Next, you’ll add a new file to this repository.

1. Click the **New file** button at the top of the **Source** page.
2. Give the file a filename of **contributors.txt**.
3. Enter your name in the empty file space.
4. Click **Commit** and then **Commit** again in the dialog.
5. Go back to the **Source** page.

Before you move on, go ahead and explore the repository. You've already seen the **Source** page, but check out the **Commits**, **Branches**, and **Settings** pages.

---

## Clone a repository

Use these steps to clone from SourceTree, our client for using the repository command-line free. Cloning allows you to work on your files locally. If you don't yet have SourceTree, [download and install first](https://www.sourcetreeapp.com/). If you prefer to clone from the command line, see [Clone a repository](https://confluence.atlassian.com/x/4whODQ).

1. You’ll see the clone button under the **Source** heading. Click that button.
2. Now click **Check out in SourceTree**. You may need to create a SourceTree account or log in.
3. When you see the **Clone New** dialog in SourceTree, update the destination path and name if you’d like to and then click **Clone**.
4. Open the directory you just created to see your repository’s files.

Now that you're more familiar with your Bitbucket repository, go ahead and add a new file locally. You can [push your change back to Bitbucket with SourceTree](https://confluence.atlassian.com/x/iqyBMg), or you can [add, commit,](https://confluence.atlassian.com/x/8QhODQ) and [push from the command line](https://confluence.atlassian.com/x/NQ0zDQ).