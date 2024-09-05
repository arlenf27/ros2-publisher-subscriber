# ROS2 Basic Image Publisher and Subscriber
This program is a basic ROS2 image publisher and subscriber. The subscriber will continuously display a new window with the exact copy of the published image. 
This program was only tested on Ubuntu 22 ARM64!

## Running the Built Program
1. Make sure that ROS2 is installed properly. 
2. Clone this repository. 
3. Navigate to the root of the repository. 
4. Run `. install/setup.bash`. 
5. Run `ros2 run cpp_pubsub talker [NAME_OF_IMAGE_FILE.jpg or NAME_OF_IMAGE_FILE.jpeg]` (Other image types have not been tested, but may work)
6. While the talker (publisher) is running, open a new terminal instance/tab and navigate to the same root directory. 
7. Run `ros2 run cpp_pubsub listener`. The image should open in a new window and should continuously open after the window is closed. 
8. Use `Ctrl-C` to terminate both the listener and talker. 

## Building the Program
1. Make sure that ROS2 is installed properly. 
2. Clone this repository. 
3. Navigate to the root of the repository. 
4. Check and install any missing dependencies: `rosdep install -i --from-path src --rosdistro humble -y`
5. Build the program: `colcon build --packages-select cpp_pubsub`
6. Follow steps 3 - 8 of "Running the Built Program" section to run the built program. 

## How This Program Was Developed
Note: Most of the "handwritten" code is in "/src/cpp_pubsub/src/". The two C++ files are the [publisher] (/src/cpp_pubsub/src/publisher_member_function.cpp) and [subscriber] (/src/cpp_pubsub/src/subscriber_member_function.cpp) source code. The CMakeLists.txt and package.xml files are located in "/src/cpp_pubsub/". 
1. The base of this program is from the ROS2.org tutorial at [docs.ROS.org: Writing a simple publisher and subscriber (C++)] (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). When finished with this tutorial, a working STRING publisher and subscriber were developed. 
2. Additional code was added and original code was modified to develop an image publisher and subscriber. 
3. Additional dependencies not used in the original tutorial are as follows: **Image_transport**, **CV_bridge**, **OpenCV**. 
4. Many resources were referenced; some of the most relevant resources are as follows: 
- [cv_bridge Namespace Reference] (https://docs.ros.org/en/noetic/api/cv_bridge/html/c++/namespacecv__bridge.html#aafa38a1d9be98d9efaefe45fd873133c)
- [cv_bridge::CvImage Class Reference] (https://docs.ros.org/en/noetic/api/cv_bridge/html/c++/classcv__bridge_1_1CvImage.html#a943bd351cfeba6e67a0b961e3b4a1ba9)
- [image_transport::ImageTransport Class Reference] (https://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1ImageTransport.html#aa66ce930baa92b21a84956b289340671)
- [OpenCV Image file reading and writing] (https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#gacbaa02cffc4ec2422dfa2e24412a99e2)
- [Migration guide from ROS 1] (https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html)
5. package.xml and CMakeLists.txt were filled with the additional dependencies to be installed/found. 
