# Sushi Tracking Library

The Sushi Tracking Library is part of my Master Thesis. It takes care of retrieving a live video of a mouth from a Raspberry, process every single frame and extract some feature points of the lips.

## How it works

It is divided into four components:

- raspberry.cpp : Here is the code that connects to the raspberry and starts the streaming.
- socket.cpp : The class that hides the native implementation of the socket functions.
- getVideo.cpp : The part of the program responsible for receiving the video through a socket.
- tracking.cpp : This is the core of the library, where is possible to find all the functions that process the image and extract the feature points.

The library assumes the camera to be fixed near to the mouth and does not provides mechanism to extract feature points from different angle or from image of the whole face.

## What is needed

The library as been wrote with Visual Studio in a Windows environment, so there are some part of the code (outside the tracking.cpp file) that could be run only on Windows. It is indeed possible to modify them without changing the core algorithm to export the library on a different system. 

It uses two extern library: OpenCV and FFMpeg. These libraris are linked as shared library, so the .lib and .dll (in case of a Windows system) files of both the library are needed to correctly run the code. As example, the .lib and .dll files of Windows10 x64 compiled in Release mode have been left within the folders of the project. The OpenCV library has been compiled with IPP and TBB. 