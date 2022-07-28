#!/usr/bin/env python

# Copyright (c) 2021 IBISC Laborartory, Pelvoux, France

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import cv2
import csv
import argparse
import collections
import time
from datetime import datetime
import logging
import math
import random
import re
import weakref
import open3d as o3d
from matplotlib import cm

try:
    sys.path.append(glob.glob('../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_k
    from pygame.locals import K_j
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError(
        'cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue
    
# ==============================================================================
# -- Starting the client -------------------------------------------------------
# ==============================================================================

client = carla.Client('localhost', 2000)
client.set_timeout(200.0)

# ==============================================================================
# -- Creating the World --------------------------------------------------------
# ==============================================================================

global world
global m
world = client.get_world()
m = world.get_map()
self = world
self.recording = False

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses

# ==============================================================================
# -- Output files Handling------------------------------------------------------
# ==============================================================================

try:
    parent_dir = os.getcwd()
    path1 = os.path.join(parent_dir, "LiDAR_Odometry_i_%s" % (m.name))
    os.mkdir(path1)
    path4 = os.path.join(path1, "rgbL")
    os.mkdir(path4)
    path8 = os.path.join(path4, "frames")
    os.mkdir(path8)
    path12 = os.path.join(path1, "rgbR")
    os.mkdir(path12)
    path15 = os.path.join(path12, "frames")
    os.mkdir(path15)
    path10 = os.path.join(path1, "other_sensors")
    os.mkdir(path10)
    path11 = os.path.join(path1, "vehicle_gt")
    os.mkdir(path11)
    path40 = os.path.join(path1, "lidar")
    os.mkdir(path40)
except OSError:
    print("Creation of the directory %s failed" % path1)
    print("Creation of the directory %s failed" % path4)
    print("Creation of the directory %s failed" % path8)
    print("Creation of the directory %s failed" % path10)
    print("Creation of the directory %s failed" % path11)
    print("Creation of the directory %s failed" % path12)
    print("Creation of the directory %s failed" % path15)
    print("Creation of the directory %s failed" % path40)
else:
    print("Successfully created the directory %s " % path1)
    print("Successfully created the directory %s " % path4)
    print("Successfully created the directory %s " % path8)
    print("Successfully created the directory %s " % path10)
    print("Successfully created the directory %s " % path11)
    print("Successfully created the directory %s " % path12)
    print("Successfully created the directory %s " % path15)
    print("Successfully created the directory %s " % path40)

file_rgbL = open(os.path.join(path4, 'timestamps.csv'), "a")
file_rgbR = open(os.path.join(path12, 'timestamps.csv'), "a")
file_groundtruth_sync = open(os.path.join(path11, 'groundtruth_sync.csv'), "a")
file_GNSS = open(os.path.join(path10, 'gnss.csv'), "a")

writer0 = csv.writer(file_rgbL, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer0.writerow(['#timestamp [ns]', 'filename'])
writer8 = csv.writer(file_rgbR, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer8.writerow(['#timestamp [ns]', 'filename'])
writer4 = csv.writer(file_groundtruth_sync, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer4.writerow(['#timestamp [ns]', 'p_RS_R_x_m_', 'p_RS_R_y_m_', 'p_RS_R_z_m_', 'v_RS_R_x_mS__1_', 'v_RS_R_y_mS__1_', 'v_RS_R_z_mS__1_', 'a_RS_S_x_mS__2_', 'a_RS_S_y_mS__2_', 'a_RS_S_z_mS__2_', 'q_RS_w__', 'q_RS_x__', 'q_RS_y__', 'q_RS_z__', 'w_RS_S_x_radS__1_', 'w_RS_S_y_radS__1_', 'w_RS_S_z_radS__1_'])
writer5 = csv.writer(file_GNSS, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer5.writerow(['#timestamp [ns]', 'latitude', 'longitude', 'altitude'])

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class gnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        bp.set_attribute('noise_alt_bias', '0.0')
        bp.set_attribute('noise_lat_bias', '0.0')
        bp.set_attribute('noise_lon_bias', '0.0')
        bp.set_attribute('noise_alt_stddev', '0.0')
        bp.set_attribute('noise_lat_stddev', '0.0')
        bp.set_attribute('noise_lon_stddev', '0.0')
        bp.set_attribute('noise_seed', '0')
        bp.set_attribute('sensor_tick', '0.05')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(
            x=-0.2, y=0, z=2.8), carla.Rotation(pitch=0, roll=0, yaw=0)), attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: gnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude
        self.tmp = event.timestamp

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()


# ==============================================================================
# -- Useful Functions-----------------------------------------------------------
# ==============================================================================
def resize_img(img):
    scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    quat = [qw, qx, qy, qz]
    return quat/np.linalg.norm(quat)


def draw_rgbL_image(surface, image_rgbL):
    array = np.frombuffer(image_rgbL.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image_rgbL.height, image_rgbL.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(resize_img(array.swapaxes(0, 1)))
    # image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))

def draw_rgbR_image(surface, image_rgbR):
    array = np.frombuffer(image_rgbR.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image_rgbR.height, image_rgbR.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(resize_img(array.swapaxes(0, 1)))
    # image_surface.set_alpha(100)
    surface.blit(image_surface, (512, 0))
    
def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def create_main_actors(start_pose, waypoint):
    blueprint_library = world.get_blueprint_library()

    vehicle = world.spawn_actor(
        blueprint_library.filter('vehicle')[1], start_pose)
    vehicle.set_simulate_physics(True)
    vehicle.set_transform(waypoint.transform)

    bp_rgb = blueprint_library.find('sensor.camera.rgb')
    # Modify the basic attributes of the blueprint
    bp_rgb.set_attribute('image_size_x', '1024')
    bp_rgb.set_attribute('image_size_y', '1024')
    bp_rgb.set_attribute('sensor_tick', '0.05')
    bp_rgb.set_attribute('bloom_intensity', '0.675')
    bp_rgb.set_attribute('fov', '90.0')
    bp_rgb.set_attribute('fstop', '1.4')
    bp_rgb.set_attribute('iso', '100.0')
    bp_rgb.set_attribute('gamma', '2.2')
    bp_rgb.set_attribute('lens_flare_intensity', '0.1')
    bp_rgb.set_attribute('shutter_speed', '200.0')
    # Modify the lens distortion attributes
    bp_rgb.set_attribute('lens_circle_falloff', '5.0')
    bp_rgb.set_attribute('lens_circle_multiplier', '0.0')
    bp_rgb.set_attribute('lens_k', '0.0')
    bp_rgb.set_attribute('lens_kcube', '0.0')
    bp_rgb.set_attribute('lens_x_size', '0.08')
    bp_rgb.set_attribute('lens_y_size', '0.08')
    camera_rgbL = world.spawn_actor(
        bp_rgb,
        carla.Transform(carla.Location(x=0.0, y=-0.2, z=2.8),
                        carla.Rotation(pitch=0.0, roll=0.0, yaw=0.0)),
        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        
    bp_rgb = blueprint_library.find('sensor.camera.rgb')
    # Modify the basic attributes of the blueprint
    bp_rgb.set_attribute('image_size_x', '1024')
    bp_rgb.set_attribute('image_size_y', '1024')
    bp_rgb.set_attribute('sensor_tick', '0.05')
    bp_rgb.set_attribute('bloom_intensity', '0.675')
    bp_rgb.set_attribute('fov', '90.0')
    bp_rgb.set_attribute('fstop', '1.4')
    bp_rgb.set_attribute('iso', '100.0')
    bp_rgb.set_attribute('gamma', '2.2')
    bp_rgb.set_attribute('lens_flare_intensity', '0.1')
    bp_rgb.set_attribute('shutter_speed', '200.0')
    # Modify the lens distortion attributes
    bp_rgb.set_attribute('lens_circle_falloff', '5.0')
    bp_rgb.set_attribute('lens_circle_multiplier', '0.0')
    bp_rgb.set_attribute('lens_k', '0.0')
    bp_rgb.set_attribute('lens_kcube', '0.0')
    bp_rgb.set_attribute('lens_x_size', '0.08')
    bp_rgb.set_attribute('lens_y_size', '0.08')
    camera_rgbR = world.spawn_actor(
        bp_rgb,
        carla.Transform(carla.Location(x=0.0, y=0.2, z=2.8),
                        carla.Rotation(pitch=0.0, roll=0.0, yaw=0.0)),
        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

    Gnss_sensor = gnssSensor(vehicle)

    return vehicle, camera_rgbL, camera_rgbR, Gnss_sensor


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

       
def _RGBL_sensor_callback(sensor_data, ts, display):
    draw_rgbL_image(display, sensor_data)
    if self.recording:
        sensor_data.save_to_disk('LiDAR_Odometry_i_%s/rgbL/frames/%d.png' % (m.name, ts))
        file_rgbL.write("%d,%d.png \n" % (ts, ts))
        
def _RGBR_sensor_callback(sensor_data, ts, display):
    draw_rgbR_image(display, sensor_data)
    if self.recording:
        sensor_data.save_to_disk('LiDAR_Odometry_i_%s/rgbR/frames/%d.png' % (m.name, ts))
        file_rgbR.write("%d,%d.png \n" % (ts, ts))

def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Isolate the 3D data
    points = data[:, :-1]

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, :1] = -points[:, :1]

    # # An example of converting points from sensor to vehicle space if we had
    # # a carla.Transform variable named "tran":
    # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
    # points = np.dot(tran.get_matrix(), points.T).T
    # points = points[:, :-1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    if self.recording:
        o3d.io.write_point_cloud('LiDAR_Odometry_i_%s/lidar/%d.pcd' % (m.name, point_cloud.timestamp*(10**9)),point_list)


def semantic_lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with semantic segmentation
    colors ready to be consumed by Open3D"""
    data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points = np.array([data['x'], -data['y'], data['z']]).T

    # # An example of adding some noise to our data if needed:
    # points += np.random.uniform(-0.05, 0.05, size=points.shape)

    # Colorize the pointcloud based on the CityScapes color palette
    labels = np.array(data['ObjTag'])
    int_color = LABEL_COLORS[labels]

    # # In case you want to make the color intensity depending
    # # of the incident ray angle, you can use:
    int_color *= np.array(data['CosAngle'])[:, None]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    if self.recording:
        o3d.io.write_point_cloud('LiDAR_Odometry_i_%s/lidar/%d.pcd' % (m.name, point_cloud.timestamp*(10**9)),point_list)


def generate_lidar_bp(arg, world, blueprint_library, delta):
    """Generates a CARLA blueprint based on the script parameters"""
    if arg.semantic:
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    else:
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        if arg.no_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        else:
            lidar_bp.set_attribute('noise_stddev', '0.2')

    lidar_bp.set_attribute('upper_fov', str(arg.upper_fov))
    lidar_bp.set_attribute('lower_fov', str(arg.lower_fov))
    lidar_bp.set_attribute('channels', str(arg.channels))
    lidar_bp.set_attribute('range', str(arg.range))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(arg.points_per_second))
    return lidar_bp


def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)    
# ==============================================================================
# -- Sensor Synchronization-----------------------------------------------------
# ==============================================================================


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data
    
# ==============================================================================
# -- Main Function--------------------------------------------------------------
# ==============================================================================


def main(arg):
    actor_list = []
    pygame.init()
    pygame.display.set_caption(
        'IBISCape: Multi-modal Data Acquisition Framework')

    display = pygame.display.set_mode(
        (1024, 512),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    iterator = 1
    rvrs = False
    hand_brake = False    

    try:
        start_Pose = m.get_spawn_points()
        start_pose = start_Pose[32]
        waypoint = m.get_waypoint(start_pose.location)
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.no_rendering_mode = arg.no_rendering

        vehicle, camera_rgbL, camera_rgbR, Gnss_sensor = create_main_actors(start_pose, waypoint)
        file_veh_sim = open(os.path.join(path11, '%s_simulation.txt' %
                                         get_actor_display_name(vehicle, truncate=200)), "a")
        # LiDAR spawning
        delta = 0.05
        blueprint_library = world.get_blueprint_library()
        lidar_bp = generate_lidar_bp(arg, world, blueprint_library, delta)
        user_offset = carla.Location(arg.x, arg.y, arg.z)
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        point_list = o3d.geometry.PointCloud()
        
        actor_list.append(vehicle)
        actor_list.append(camera_rgbL)
        actor_list.append(camera_rgbR)
        actor_list.append(Gnss_sensor)
        actor_list.append(lidar)
        
        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_rgbL, camera_rgbR, lidar, fps=20) as sync_mode:

            # LiDAR Visualization setup
            #vis = o3d.visualization.Visualizer()
            #vis.create_window(
            #    window_name='IBISCape LiDAR',
            #    width=960,
            #    height=540,
            #    left=480,
            #    top=270)
            #vis.get_render_option().background_color = [0.05, 0.05, 0.05]
            #vis.get_render_option().point_size = 1
            #vis.get_render_option().show_coordinate_frame = True
            #if arg.show_axis:
            #    add_open3d_axis(vis)
        
            # Weather
            world.set_weather(carla.WeatherParameters.HardRainSunset)  #ClearNoon, CloudyNoon, WetNoon, WetCloudyNoon, SoftRainNoon, MidRainyNoon, HardRainNoon, ClearSunset, CloudySunset, WetSunset, WetCloudySunset, SoftRainSunset, MidRainSunset, HardRainSunset, WetCloudySunset
        
            frame = 0
            dt0 = datetime.now()
            while True:

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == K_r:
                            self.recording = not self.recording
                        elif event.key == K_q:
                            rvrs = not rvrs
                        elif event.key == K_SPACE:
                            vehicle.disable_constant_velocity()
                            vehicle.apply_control(
                                carla.VehicleControl(hand_brake=not hand_brake))
                        elif event.key == K_w:
                            if rvrs:
                                vehicle.apply_control(
                                    carla.VehicleControl(reverse=True, brake=0.0))
                                vehicle.enable_constant_velocity(
                                    carla.Vector3D(-3, 0, 0))
                            else:
                                vehicle.apply_control(
                                    carla.VehicleControl(throttle=1.0, brake=0.0))
                                vehicle.enable_constant_velocity(
                                    carla.Vector3D(3, 0, 0))
                        elif event.key == K_s:
                            vehicle.disable_constant_velocity()
                            vehicle.apply_control(
                                carla.VehicleControl(throttle=0.0, brake=1.0))
                        elif event.key == K_a:
                            vehicle.apply_control(
                                carla.VehicleControl(steer=-1.0))
                        elif event.key == K_d:
                            vehicle.apply_control(
                                carla.VehicleControl(steer=1.0))
                        elif event.key == pygame.K_ESCAPE:
                            return
                    elif event.type == pygame.KEYUP:
                        if event.key == K_a:
                            vehicle.apply_control(
                                carla.VehicleControl(steer=0.0))
                        elif event.key == K_d:
                            vehicle.apply_control(
                                carla.VehicleControl(steer=0.0))

                clock.tick()

                # Advance the simulation and wait for the data.
                snapshot, image_rgbL, image_rgbR, LiDAR_data = sync_mode.tick(timeout=2.0)
                Pose = vehicle.get_transform()
                Accl = vehicle.get_acceleration()
                velo = vehicle.get_velocity()
                AngV = vehicle.get_angular_velocity()
                Ctrl = vehicle.get_control()
                t_world = snapshot.timestamp.elapsed_seconds*(10**9)

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                _RGBL_sensor_callback(image_rgbL, t_world, display)
                _RGBR_sensor_callback(image_rgbR, t_world, display)
                if arg.semantic:
                    semantic_lidar_callback(LiDAR_data, point_list)
                else:
                    lidar_callback(LiDAR_data, point_list)               
                
                if self.recording:
                    file_GNSS.write("%f,%21.21f,%21.21f,%21.21f \n" % (t_world, Gnss_sensor.lat, Gnss_sensor.lon, Gnss_sensor.alt))
                    quat = euler_to_quaternion(math.radians(Pose.rotation.yaw), math.radians(Pose.rotation.pitch), math.radians(Pose.rotation.roll))
                    file_groundtruth_sync.write("%d,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f,%21.21f \n" % (t_world, Pose.location.x, Pose.location.y, Pose.location.z, velo.x, velo.y, velo.z, Accl.x, Accl.y, Accl.z, quat[0], quat[1], quat[2], quat[3], math.radians(AngV.x), math.radians(AngV.y), math.radians(AngV.z)))
                    file_veh_sim.write("%d,%f,%f,%f \n" % (
                        t_world, Ctrl.throttle, Ctrl.steer, Ctrl.brake))
                    display.blit(font.render('Calibration Frame Number: %d' % (
                        iterator), True, (250, 0, 0)), (50, 208))
                    iterator += 1

                display.blit(
                    font.render('% 5d FPS (real)' %
                                clock.get_fps(), True, (250, 0, 0)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' %
                                fps, True, (250, 0, 0)),
                    (8, 28))
                display.blit(font.render('Recording %s' % (
                    'On' if self.recording else 'Off - Press R Key to Record'), True, (250, 0, 0)), (50, 46))
                display.blit(font.render('GNSS:% 24s' % ('(% 3.8f, % 3.8f, % 3.8f)' % (
                    Gnss_sensor.lat, Gnss_sensor.lon, Gnss_sensor.alt)), True, (250, 0, 0)), (50, 100))
                display.blit(font.render('x=%5.8f,y=%5.8f,z=%5.8f (m)' % (
                    Pose.location.x, Pose.location.y, Pose.location.z), True, (250, 0, 0)), (50, 118))
                display.blit(font.render('vx=%5.8f,vy=%5.8f,vz=%5.8f (m/sec)' % (
                    velo.x, velo.y, velo.z), True, (250, 0, 0)), (50, 136))
                display.blit(font.render('ax=%5.8f,ay=%5.8f,az=%5.8f (m/sec^2)' % (
                    Accl.x, Accl.y, Accl.z), True, (250, 0, 0)), (50, 154))
                display.blit(font.render('roll=%5.8f,pitch=%5.8f,yaw=%5.8f (rad)' % (
                    math.radians(Pose.rotation.roll), math.radians(Pose.rotation.pitch), math.radians(Pose.rotation.yaw)), True, (250, 0, 0)), (50, 172))
                display.blit(font.render('r_d=%5.8f,p_d=%5.8f,y_d=%5.8f (rad/sec)' % (
                    math.radians(AngV.x), math.radians(AngV.y), math.radians(AngV.z)), True, (250, 0, 0)), (50, 190))
                # LiDAR sensor DATA
                #if frame == 2:
                #    vis.add_geometry(point_list)
                #vis.update_geometry(point_list)
                #vis.poll_events()
                #vis.update_renderer()
                # # This can fix Open3D jittering issues:
                #time.sleep(0.005)
                process_time = datetime.now() - dt0
                sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
                sys.stdout.flush()
                dt0 = datetime.now()
                frame += 1
                pygame.display.flip()

    finally:

        print('\nDestroying actors.')
        for actor in actor_list:
            actor.destroy()
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)
        file_rgbL.close()
        file_rgbR.close()
        file_groundtruth_sync.close()
        file_GNSS.close()
        file_veh_sim.close()
        #vis.destroy_window()
        pygame.quit()
        print('done.')


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='use the no-rendering mode which will provide some extra'
        ' performance but you will lose the articulated objects in the'
        ' lidar, such as pedestrians')
    argparser.add_argument(
        '--semantic',
        action='store_true',
        help='use the semantic lidar instead, which provides ground truth'
        ' information')
    argparser.add_argument(
        '--no-noise',
        action='store_true',
        help='remove the drop off and noise from the normal (non-semantic) lidar')
    argparser.add_argument(
        '--no-autopilot',
        action='store_false',
        help='disables the autopilot so the vehicle will remain stopped')
    argparser.add_argument(
        '--show-axis',
        action='store_true',
        help='show the cartesian coordinates axis')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='model3',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--upper-fov',
        default=15.0,
        type=float,
        help='lidar\'s upper field of view in degrees (default: 15.0)')
    argparser.add_argument(
        '--lower-fov',
        default=-25.0,
        type=float,
        help='lidar\'s lower field of view in degrees (default: -25.0)')
    argparser.add_argument(
        '--channels',
        default=64.0,
        type=float,
        help='lidar\'s channel count (default: 64)')
    argparser.add_argument(
        '--range',
        default=100.0,
        type=float,
        help='lidar\'s maximum range in meters (default: 100.0)')
    argparser.add_argument(
        '--points-per-second',
        default=5000000,
        type=int,
        help='lidar\'s points per second (default: 500000)')
    argparser.add_argument(
        '-x',
        default=0.0,
        type=float,
        help='offset in the sensor position in the X-axis in meters (default: 0.0)')
    argparser.add_argument(
        '-y',
        default=0.0,
        type=float,
        help='offset in the sensor position in the Y-axis in meters (default: 0.0)')
    argparser.add_argument(
        '-z',
        default=3.0,
        type=float,
        help='offset in the sensor position in the Z-axis in meters (default: 0.0)')
    args = argparser.parse_args()
    
    try:

        main(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
