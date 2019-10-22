# RCAndVRSupportForDrones
Source code for my master thesis

Detailed instructions are provided in sub-readmes

We provided a robust and scalable solution for communication based on MAVLink protocol between DJI drones, its onboard computer and an Android tablet computer, where the telecommunication link is established over the remote controller. We tested the range and reliability of our framework in simulated and real-life environments, where it proved fully capable of steering a drone on flight missions.  We also did a proof of concept integration with an existing ground control station. 

To allow for additional methods of visualization and manipulation, we also developed a fully ROS-integrated tool based on the OpenVR API, that provides support for most VR headsets and controllers through RViz visualizer. We furthermore looked into using that for adaptive view management in an exploration scenario, where the drone operator is provided an overview of the area of interest through a secondary virtual camera or drone.
