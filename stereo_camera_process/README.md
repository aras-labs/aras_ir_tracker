# Stereo Camera Pipeline

The online and offline reconstruction and calibration processes for a stereo setup are hosted in this folder:
- **calibration.ipynb** A notebook that implements the multi-camera calibration procedure as explaned [here](/doc/tutorial2_stereo_3d.md).
- **reconstruction.ipynb** A notebook that uses the outputs of the calibration stage and reconstructs the 3D locations of the markers as explaned [here](/doc/tutorial2_stereo_3d.md).
- **stereo_tracker_3d.py** An online implementation of the stereo 3D tracker. The script receives marker observations from the udp_aggregator and the path to the stereo intrinsic and extrinsic parameters and computes the 3D position of the observed marker.
