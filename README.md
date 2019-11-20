# RCAndVRSupportForDrones
Source code for my master thesis.

Detailed instructions are provided in sub-readmes.

My master's thesis provides with two contributions:
- A communication protocol based on MAVLink between an Android device and a DJI drone's onboard ROS computer
- A virtual reality plugin based on OpenVR for ROS, which runs natively in RViz's OGRE engine

It was done under mentorship of Prof. Friedrich Fraundorfer and Prof. Matija Marolt, with the tutelage of assistants Jesús Pestana Puerta and Alexander W. Isop.

Link to Google Drive, containing a few videos of code in action: https://drive.google.com/drive/folders/1_dw5pWfECyFT5N4Vk5lfcv2X-rXcRgOX?usp=sharing

We provided a robust and scalable solution for communication based on MAVLink protocol between DJI drones, its onboard computer and an Android tablet computer, where the telecommunication link is established over the remote controller. We tested the range and reliability of our framework in simulated and real-life environments, where it proved fully capable of steering a drone on flight missions.  We also did a proof of concept integration with an existing ground control station. 

![image](https://user-images.githubusercontent.com/56850221/67252800-dcd76c80-f474-11e9-9d29-780a3a2a44bb.png)


To allow for additional methods of visualization and manipulation, we also developed a fully ROS-integrated tool based on the OpenVR API, that provides support for most VR headsets and controllers through RViz visualizer. We furthermore looked into using that for adaptive view management in an exploration scenario, where the drone operator is provided an overview of the area of interest through a secondary virtual camera or drone.

![image](https://user-images.githubusercontent.com/56850221/67252709-73eff480-f474-11e9-9196-10572b0c7685.png)
