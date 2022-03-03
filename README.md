# IBISCape: Multimodal  Frameworks  for  Heterogeneous  Data Acquisition  in  Dynamic  Environments 

<p align="center">
    <img src="IBISCape_github.gif" alt="Video to Events" width="800"/>
</p>

To run any of our acquisition frameworks, make sure you have: 
- CARLA installed version up to 0.9.11 [CARLA installation repository](https://github.com/carla-simulator/carla.git).
> IBISCape frameworks are tested on CARLA versions: 0.9.10, 0.9.10.1, and best performance with 0.9.11.
- OpenCV any version [OpenCV installation repository](https://github.com/opencv/opencv.git).
- Clone our IBISCape repository in a folder next to your CARLA installation directory and check the (.egg) file imported in all the frameworks to be similar to your installation [Check this link](https://carla.readthedocs.io/en/latest/build_system/#versions-prior-to-0912).
## Calibration Acquisition frameworks 
- Our new CARLA-town and 2 sample calibration sequences can be downloaded from this [link](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/EuomVZ3huEVCranEAQYEk-IBI36KQdZy9vov9wftZ7IQwQ?e=m5RM2j).
- Add our CARLA calibration town to the Map content folder to your CARLA installation directory `/your_CARLA_directory/carla/Unreal/CarlaUE4/Content/Carla/Maps/`.
- Then, open the CARLA-Unreal Engine interface and press Compile, Build and finally Play.
- Run one of our 4 data acquisition frameworks: main_`(mono-RGB/Sync)`, `(mono-RGB/Async)`, `(stereo-RGB/Sync)`, and `(stereo-RGB/Async)`. 
```bash
$ python3 framework_name.py
```
- We performed the monocular RGB-IMU calibration using [kalibr](https://github.com/ethz-asl/kalibr.git), and all the configuration .yaml files are added to the yaml_config_files folder. (Also, configuration files for [OKVIS](https://github.com/ethz-asl/okvis.git) and [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3.git))
> The calibration data acquisition is done manually, where the user can collect sequences with all the movements specific to his application.
## CARLA-IMU initial Calibration 
- Add the IMU.csv file and the groundtruth.csv file with the same naming as the samples you can find in the `/IMU_init_calib/data folder`.
- Then, run `/IMU_init_calib/apps/MAIN_IMU_BsplineV2.m` for IBISCape sequences or `/IMU_init_calib/apps/MAIN_IMU_Bspline.m` for EuRoC sequences.
- The calibration is done based on the RMSE minimization optimization problem and a complete report (.txt file) and figures (.pdf) are generated automatically in the same apps folder.
## VIO-SLAM Acquisition frameworks 
- Play your desired CARLA-town, then run one of our 4 data acquisition frameworks: main_`(mono-RGB/Sync)`, `(mono-RGB/Async)`, `(stereo-RGB/Sync)`, and `(stereo-RGB/Async)`.
```bash
$ python3 framework_name.py
```
- All sensors readings in the stereo-RGB are timestamped with the world tick, while for the mono-RGB the readings are timestamped according to the sensor local tick.
- To add people and cars to the scene, run the spawn_npc.py script in a new terminal and then close it ctrl+c. This will spawn people and cars to your Carla-client as additional actors.
- 30 sample odometry and SLAM sequences for all available CARLA-towns in highly dynamic weather can be downloaded from these links: [mono-RGB](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/EodU1MCvDDNKrB-CyHoXFKMB_nyLzKbM9Ojt65YrsJ-JOg?e=AASF3k), and [stereo-RGB](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/EmnWbAaarkRAm5TDh1hgs_cBKJgMgKZADlRS-0kIziD0VA?e=4gJvYV).
> The VIO-SLAM data acquisition is done using CARLA autopilot, following its traffic manager to perfectly simulate the real world.
## IBISCape ROS_tools
- Here we developed some python scripts to help in creating ROS bags for ROS-based calibration and odometry frameworks with the right and synchronized timestamping.
- Also, some additional scripts to read the rosbags, convert all RGB images in sequence folder to grayscale images, and associating the RGB with the corresponding depth maps.
