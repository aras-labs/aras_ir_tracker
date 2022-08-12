# ARAS IR-Tracker
## Introduction

ARAS IR-Tracker is a modular infrared motion capture system for robotic and healthcare applications. It is developed and built at [Advanced Robotics and Automation Systems lab ](aras.kntu.ac.ir) at the K.N.Toosi University of Technology in Tehran (link). The system is designed to be modular with each module encapsulating isolated functionalities.

This project is in active development, and currently supports stereo and multi-camera tD tracking of a single marker. Support for full 6DoF tracking and IMU fusion for improved robustness are on the list of future works. 

## System Architecture

![System Overall Architecture](/doc/system_architecture.png)

As shown above, the system is comprised of:

- **Marker Tracker Cameras:** Camera platforms with or without onboard processing components. They are responsible for tracking the 2D pixel positions of detected markers in the image to the server based on a unified UDP interface. For optimal accuracy in tracking dynamic markers, the cameras can be synchronized using the [ARAS ESP Sync Generator and NTP Server](void). 

- **UDP Aggregator:**: A process that runs on the server and aggregates (and synchronizes) the marker observations UDP packets from the camera nodes.

- **Multi/Stereo Calibration and Reconstruction Nodes:**: Shown with blue boxes, these nodes use a dataset recorded by the UDP aggregator node to estimate the relative poses of the cameras (system extrinsic parameters) or the marker poses in an offline setting. The data pipeline for this process is indicated with dashed arrows which means that it is a one-time requirement. 

- **Multi/Stereo Tracker Node:** A process on the server that receives the aggregated marker observations from the UDP aggregator node and the camera intrinsic and extrinsic parameters from the offline [calibration process](void) and estimates the 3D/6D marker positions/poses.

- **ESP32 Sync Pulse Generator and NTP Server (Optional):** A locally developed system that gets the time from an accurate source such as a GPS or RTC chip and runs an NTP UDP server for the system. Additionally, this system can generate stamped camera trigger pulses for sensor synchronization. Instead of using this system, one can synchronize the cameras against the FSYNC output of one of the cameras and use Linux NTP to synchronize all the local clocks against one of the PCs in the system.

## How does it Work?

This section provides a high-level and simplified explanation of the working principle of the system. For more detail, the reader may refer to the corresponding publications [here](#citation). 

The system aims to accurately track the 3D positions of one or several markers in the scene (the scene is an area where cameras have intersecting fields of view). The markers can be active (powered IR LEDs) or passive (retroreflective markers and IR projectors installed on each camera). 

All cameras point to a common area in the workspace and their relative poses with respect to each other are known (estimated during our [calibration process]()). At each point in time, the observed markers in each camera frame are tranced in the form of rays in the 3D world. The intersections of these rays (in the least square sense) are candidate 3D positions that correspond to markers (there can be ambagious situations that should be resolved). This has been illustrated below for a stereo setup:

<p align="center">
  <img width="460" height="300" src="/doc/stereo_ray_intersection.jpg">
</p>

For a multi-camera system, this intersection is like the following image (image taken from the [OptiTrack documentation](https://v22.wiki.optitrack.com/index.php?title=Reconstruction_and_2D_Mode)):

<p align="center">
  <img width="460" height="300" src="/doc/optitrack_rays.png">
</p>

The output of the system after this stage is a single marker location (the current version of the system) or a full 6D pose that represents the 6D configuration of several markers.

## Usage
Based on the chosen camera hardware, required workspace, and the speed of the markers that need to be tracked, the system can have various realizations. Here, we enlist representative examples that should get your hand dirty enough to improvise on your own! 

Initially and if you want to get a sense of how the system works, you can use our sample datasets to run the online trackers nodes and offline reconstruction/calibration Jupyter notebooks. For further details, refer to the following tutorials:

- [Tutorial #1: Multi-View Calibration and Tracking (3D)](/doc/tutorial1_multi_3d.md)

- [Tutorial #2: Stereo Calibration and Tracking (3D)](/doc/tutorial2_stereo_3d.md)

- [Tutorial #3: PS3-Eye Sensor Hardware and Software Setup](/marker_trackers/ps3_eye/README.md)

More examples and tutorials are to be added in the future ... 
## Citation

For further details on the theoretical principles of the system, you can refer to the following publications. If you use this system for your research, we would appreciate you cite us! 

```latex
@inproceedings{damirchi2019aras,
  title={ARAS-IREF: An Open-Source Low-Cost Framework for Pose Estimation},
  author={Damirchi, H and Khorrambakht, R and Taghirad, HD},
  booktitle={2019 7th International Conference on Robotics and Mechatronics (ICRoM)},
  pages={303--308},
  year={2019},
  organization={IEEE}
}
```
