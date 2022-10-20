# IBISCape: A Simulated Benchmark for multi-modal SLAM Systems Evaluation in Large-scale Dynamic Environments

<p align="center">
    <img src="IBISCape_github.gif" alt="Video to Events" width="800"/>
</p>

## Citation
Are you interested to use IBISCape data acquisition frameworks, or to evaluate your new SLAM system on IBISCape benchmark in an academic work ! 
Thanks for citing the following [paper](https://link.springer.com/article/10.1007/s10846-022-01753-7):

    @Article{IBISCape22,
    author={Soliman, Abanob and Bonardi, Fabien and Sidib{\'e}, D{\'e}sir{\'e} and Bouchafa, Samia},
    title={{IBISCape}: A Simulated Benchmark for multi-modal {SLAM} Systems Evaluation in Large-scale Dynamic Environments},
    journal={Journal of Intelligent {\&} Robotic Systems},
    year={2022},
    month={Oct},
    day={19},
    volume={106},
    number={3},
    pages={53},
    issn={1573-0409},
    doi={10.1007/s10846-022-01753-7},
    url={https://doi.org/10.1007/s10846-022-01753-7}
    }

To run any of our acquisition frameworks, make sure you have: 
- CARLA installed version up to 0.9.11 [CARLA installation repository](https://github.com/carla-simulator/carla.git).
> IBISCape frameworks are tested on CARLA versions: 0.9.10, 0.9.10.1, and best performance with 0.9.11.
- OpenCV any version [OpenCV installation repository](https://github.com/opencv/opencv.git).
- Clone our IBISCape repository in a folder next to your CARLA installation directory and check the (.egg) file imported in all the frameworks to be similar to your installation [Check this link](https://carla.readthedocs.io/en/latest/build_system/#versions-prior-to-0912).
## Calibration Acquisition frameworks 
- Our new Calibration targets added to CARLA-Town3 can be downloaded from this [link](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/EhMGdMFvWtNApc8B1xJk05cBPAAvnrrdivA86QBMkddoHQ?e=rfWolS).
- Add our CARLA calibration town to the Map content folder to your CARLA installation directory `/your_CARLA_directory/carla/Unreal/CarlaUE4/Content/Carla/Maps/`.
- Then, open the CARLA-Unreal Engine interface and press Compile, Build and finally Play.
- Run one of our 6 data acquisition frameworks: `(full_sensor_setup)`, `(rgb_depth)`, `(imu)`, `(lidar_svo)`, `(stereo_dvs_imu)`, and `(stereo_rgb_imu)`. 
```bash
$ python3 framework_name.py
```
- We performed the stereo RGB-IMU calibration using [kalibr](https://github.com/ethz-asl/kalibr.git), and all the configuration .yaml files are added to the yaml_config_files folder. (Also, configuration files for [EVO](https://github.com/uzh-rpg/rpg_dvs_evo_open), [ESVO](https://github.com/HKUST-Aerial-Robotics/ESVO), [BASALT](https://github.com/VladyslavUsenko/basalt-mirror) and [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3.git))
> The calibration data acquisition is done manually, where the user can collect sequences with all the movements specific to his application.
## VI-SLAM Acquisition frameworks 
- Play your desired CARLA-town, then run one of our 5 data acquisition frameworks: `(full_sensor_setup)`, `(rgb_depth)`, `(lidar_svo)`, `(stereo_dvs_imu)`, and `(stereo_rgb_imu)`.
```bash
$ python3 framework_name.py
```
- All sensors readings in the stereo-RGB are timestamped with the world tick, while for the mono-RGB the readings are timestamped according to the sensor local tick.
- To add people and cars to the scene, run the spawn_npc.py script in a new terminal and then close it ctrl+c. This will spawn people and cars to your Carla-client as additional actors.
- The 43 calibration and SLAM sequences for all available CARLA-towns in highly dynamic weather can be downloaded from these links: [calibration](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/EpUQUsjifzFAi5Ey8kJ1uUUBG1pWVqmind6drgNvAMm0mA?e=PuiGxZ), [SLAM+Evaluation](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/Eo6fKSsfuYVGnDgh821bl90BYKThrRMbFxfdbR-Qm3tAoA?e=2lhV53), and [Configuration](https://ueve-my.sharepoint.com/:f:/g/personal/abanob_soliman_univ-evry_fr/EtECQyUEPapAmNzzQCphkQ0BQEM8DGGBzpLD181vB5WelA?e=4dDbgx).
> The VIO-SLAM data acquisition is done using CARLA autopilot, following its traffic manager to perfectly simulate the real world.
## IBISCape ROS_tools
- Here we developed some python scripts to help in creating ROS bags for ROS-based calibration and odometry frameworks with the right and synchronized timestamping.
- Also, some additional scripts to read the rosbags, convert all RGB images in sequence folder to grayscale images, and associating the RGB with the corresponding depth maps.
