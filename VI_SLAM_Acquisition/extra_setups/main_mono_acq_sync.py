#!/usr/bin/env python

# Copyright (c) 2021 IBISC Laborartory, Pelvoux, France

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import random
import glob
import os
import sys
import cv2
import csv

try:
    sys.path.append(glob.glob('../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

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
client.set_timeout(2.0)

# ==============================================================================
# -- Creating the World --------------------------------------------------------
# ==============================================================================

global world
global m
world = client.get_world()
m = world.get_map()
self = world
self.recording = False

# ==============================================================================
# -- Output files Handling------------------------------------------------------
# ==============================================================================

try:
    parent_dir = os.getcwd()
    path1 = os.path.join(parent_dir, "Sync_mono_i_%s" % (m.name))
    os.mkdir(path1)
    path2 = os.path.join(path1, "davis")
    os.mkdir(path2)
    path6 = os.path.join(path2, "frames")
    os.mkdir(path6)
    path3 = os.path.join(path1, "depth")
    os.mkdir(path3)
    path7 = os.path.join(path3, "frames")
    os.mkdir(path7)
    path4 = os.path.join(path1, "rgb")
    os.mkdir(path4)
    path8 = os.path.join(path4, "frames")
    os.mkdir(path8)
    path5 = os.path.join(path1, "semantic")
    os.mkdir(path5)
    path9 = os.path.join(path5, "frames")
    os.mkdir(path9)
    path10 = os.path.join(path1, "other_sensors")
    os.mkdir(path10)
    path11 = os.path.join(path1, "vehicle_gt")
    os.mkdir(path11)
except OSError:
    print("Creation of the directory %s failed" % path1)
    print("Creation of the directory %s failed" % path2)
    print("Creation of the directory %s failed" % path3)
    print("Creation of the directory %s failed" % path4)
    print("Creation of the directory %s failed" % path5)
    print("Creation of the directory %s failed" % path6)
    print("Creation of the directory %s failed" % path7)
    print("Creation of the directory %s failed" % path8)
    print("Creation of the directory %s failed" % path9)
    print("Creation of the directory %s failed" % path10)
    print("Creation of the directory %s failed" % path11)
else:
    print("Successfully created the directory %s " % path1)
    print("Successfully created the directory %s " % path2)
    print("Successfully created the directory %s " % path3)
    print("Successfully created the directory %s " % path4)
    print("Successfully created the directory %s " % path5)
    print("Successfully created the directory %s " % path6)
    print("Successfully created the directory %s " % path7)
    print("Successfully created the directory %s " % path8)
    print("Successfully created the directory %s " % path9)
    print("Successfully created the directory %s " % path10)
    print("Successfully created the directory %s " % path11)

file_rgb = open(os.path.join(path4, 'timestamps.csv'), "a")
file_depth = open(os.path.join(path3, 'timestamps.csv'), "a")
file_davis = open(os.path.join(path2, 'timestamps.csv'), "a")
file_sseg = open(os.path.join(path5, 'timestamps.csv'), "a")
file_groundtruth_sync = open(os.path.join(path11, 'groundtruth_sync.csv'), "a")
file_GNSS_sync = open(os.path.join(path10, 'gnss_sync.csv'), "a")
file_IMU_sync = open(os.path.join(path10, 'imu_sync.csv'), "a")
file_events = open(os.path.join(path2, 'events.csv'), "a")
file_cond = open(os.path.join(path11, 'weather_states.txt'), "a")

writer0 = csv.writer(file_rgb, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer0.writerow(['#timestamp [ns]', 'filename'])
writer1 = csv.writer(file_depth, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer1.writerow(['#timestamp [ns]', 'filename'])
writer2 = csv.writer(file_davis, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer2.writerow(['#timestamp [ns]', 'filename'])
writer3 = csv.writer(file_sseg, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer3.writerow(['#timestamp [ns]', 'filename'])
writer4 = csv.writer(file_groundtruth_sync, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer4.writerow(['#timestamp [ns]', 'p_RS_R_x_m_', 'p_RS_R_y_m_', 'p_RS_R_z_m_', 'v_RS_R_x_mS__1_', 'v_RS_R_y_mS__1_', 'v_RS_R_z_mS__1_', 'a_RS_S_x_mS__2_', 'a_RS_S_y_mS__2_', 'a_RS_S_z_mS__2_', 'q_RS_w__', 'q_RS_x__', 'q_RS_y__', 'q_RS_z__', 'w_RS_S_x_radS__1_', 'w_RS_S_y_radS__1_', 'w_RS_S_z_radS__1_'])
writer5 = csv.writer(file_GNSS_sync, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer5.writerow(['#timestamp [ns]', 'latitude', 'longitude', 'altitude'])
writer6 = csv.writer(file_IMU_sync, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer6.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
writer7 = csv.writer(file_events, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
writer7.writerow(['#timestamp', 'x', 'y', 'p'])

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
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
        #bp.set_attribute('sensor_tick', '0.1')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(
            x=-0.2, y=0, z=2.8), carla.Rotation(pitch=0, roll=0, yaw=0)), attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: GnssSensor._on_gnss_event(weak_self, event))

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
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        # Accelerometer Noise
        bp.set_attribute('noise_accel_stddev_x', '0.07')
        bp.set_attribute('noise_accel_stddev_y', '0.07')
        bp.set_attribute('noise_accel_stddev_z', '0.07')
        # Gyroscope Noise
        bp.set_attribute('noise_gyro_stddev_x', '0.004')
        bp.set_attribute('noise_gyro_stddev_y', '0.004')
        bp.set_attribute('noise_gyro_stddev_z', '0.004')
        # Noise to the biases
        bp.set_attribute('noise_gyro_bias_x', '0.0')
        bp.set_attribute('noise_gyro_bias_y', '0.0')
        bp.set_attribute('noise_gyro_bias_z', '0.0')
        bp.set_attribute('noise_seed', '0')
        #bp.set_attribute('sensor_tick', '0.005')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(carla.Location(x=0, y=0, z=0), carla.Rotation(pitch=0, roll=0, yaw=0)), attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            math.radians(max(limits[0], min(limits[1], math.degrees(
                sensor_data.gyroscope.x)))),
            math.radians(max(limits[0], min(limits[1], math.degrees(
                sensor_data.gyroscope.y)))),
            math.radians(max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z)))))
        self.compass = math.degrees(sensor_data.compass)
        self.tmp = sensor_data.timestamp

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()

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
# -- Useful Functions-----------------------------------------------------------
# ==============================================================================
def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    quat = [qw, qx, qy, qz]
    return quat/np.linalg.norm(quat)

def draw_image(surface, image_dvs, image_rgb, image_semseg, image_depth, t_world):
    dvs_events = np.frombuffer(image_dvs.raw_data, dtype=np.dtype(
        [('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
    dvs_img = np.zeros((image_dvs.height, image_dvs.width, 3), dtype=np.uint8)
    # Blue is positive, red is negative
    dvs_img[dvs_events[:]['y'], dvs_events[:]
            ['x'], dvs_events[:]['pol'] * 2] = 255
    image_surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
    # image_surface.set_alpha(100)
    surface.blit(image_surface, (512, 512))

    array = np.frombuffer(image_depth.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image_depth.height, image_depth.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    # image_surface.set_alpha(100)
    surface.blit(image_surface, (512, 0))

    array = np.frombuffer(image_rgb.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image_rgb.height, image_rgb.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    # image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))

    array = np.frombuffer(image_semseg.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image_semseg.height, image_semseg.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    # image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 512))

    if self.recording:
        outputfile = 'Sync_mono_i_%s/davis/frames/%d.png' % (m.name, t_world)
        cv2.imwrite(outputfile, dvs_img)
        image_rgb.save_to_disk('Sync_mono_i_%s/rgb/frames/%d.png' % (m.name, t_world))
        image_depth.save_to_disk('Sync_mono_i_%s/depth/frames/%d.png' % (m.name,t_world))
        image_semseg.save_to_disk('Sync_mono_i_%s/semantic/frames/%d.png' % (m.name, t_world))
        writer7.writerows(np.transpose([dvs_events[:]['t'], dvs_events[:]['x'], dvs_events[:]['y'], dvs_events[:]['pol']]))
        file_rgb.write("%d;%d.png \n" % (t_world, t_world))
        file_depth.write("%d;%d.png \n" % (t_world, t_world))
        file_sseg.write("%d;%d.png \n" % ( t_world, t_world))
        file_davis.write("%d;%d.png \n" % (t_world, t_world))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def create_main_actors(start_pose, waypoint):
    blueprint_library = world.get_blueprint_library()

    vehicle = world.spawn_actor(
        blueprint_library.filter('vehicle')[7], start_pose)
    vehicle.set_simulate_physics(True)
    vehicle.set_transform(waypoint.transform)
    # vehicle.enable_constant_velocity(carla.Vector3D(7, 0, 0))
    vehicle.set_autopilot(True)

    bp_rgb = blueprint_library.find('sensor.camera.rgb')
    # Modify the basic attributes of the blueprint
    bp_rgb.set_attribute('image_size_x', '512')
    bp_rgb.set_attribute('image_size_y', '512')
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
    bp_rgb.set_attribute('lens_k', '-1.0')
    bp_rgb.set_attribute('lens_kcube', '0.0')
    bp_rgb.set_attribute('lens_x_size', '0.08')
    bp_rgb.set_attribute('lens_y_size', '0.08')
    camera_rgb = world.spawn_actor(
        bp_rgb,
        carla.Transform(carla.Location(x=0, y=-0.2, z=2.8),
                        carla.Rotation(pitch=0, roll=0, yaw=0)),
        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

    bp_semseg = blueprint_library.find('sensor.camera.semantic_segmentation')
    # Modify the basic attributes of the blueprint
    bp_semseg.set_attribute('image_size_x', '512')
    bp_semseg.set_attribute('image_size_y', '512')
    bp_semseg.set_attribute('fov', '110')
    bp_semseg.set_attribute('sensor_tick', '0.05')
    # Modify the lens distortion attributes
    bp_semseg.set_attribute('lens_circle_falloff', '5.0')
    bp_semseg.set_attribute('lens_circle_multiplier', '0.0')
    bp_semseg.set_attribute('lens_k', '-1.0')
    bp_semseg.set_attribute('lens_kcube', '0.0')
    bp_semseg.set_attribute('lens_x_size', '0.08')
    bp_semseg.set_attribute('lens_y_size', '0.08')
    camera_semseg = world.spawn_actor(
        bp_semseg,
        carla.Transform(carla.Location(x=0, y=0, z=2.8),
                        carla.Rotation(pitch=0, roll=0, yaw=0)),
        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

    bp_depth = blueprint_library.find('sensor.camera.depth')
    # Modify the basic attributes of the blueprint
    bp_depth.set_attribute('image_size_x', '512')
    bp_depth.set_attribute('image_size_y', '512')
    bp_depth.set_attribute('fov', '110')
    bp_depth.set_attribute('sensor_tick', '0.05')
    # Modify the lens distortion attributes
    bp_depth.set_attribute('lens_circle_falloff', '5.0')
    bp_depth.set_attribute('lens_circle_multiplier', '0.0')
    bp_depth.set_attribute('lens_k', '-1.0')
    bp_depth.set_attribute('lens_kcube', '0.0')
    bp_depth.set_attribute('lens_x_size', '0.08')
    bp_depth.set_attribute('lens_y_size', '0.08')
    camera_depth = world.spawn_actor(
        bp_depth,
        carla.Transform(carla.Location(x=0, y=-0.2, z=2.8),
                        carla.Rotation(pitch=0, roll=0, yaw=0)),
        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

    bp_dvs = blueprint_library.find('sensor.camera.dvs')
    # Modify the basic attributes of the blueprint
    bp_dvs.set_attribute('image_size_x', '512')
    bp_dvs.set_attribute('image_size_y', '512')
    bp_dvs.set_attribute('fov', '110')
    bp_dvs.set_attribute('sensor_tick', '0.05')
    # Modify the lens distortion attributes
    bp_dvs.set_attribute('lens_circle_falloff', '5.0')
    bp_dvs.set_attribute('lens_circle_multiplier', '0.0')
    bp_dvs.set_attribute('lens_k', '-1.0')
    bp_dvs.set_attribute('lens_kcube', '0.0')
    bp_dvs.set_attribute('lens_x_size', '0.08')
    bp_dvs.set_attribute('lens_y_size', '0.08')
    # Modify specific DVS camera attributes
    bp_dvs.set_attribute('positive_threshold', '0.3')
    bp_dvs.set_attribute('negative_threshold', '0.3')
    bp_dvs.set_attribute('sigma_positive_threshold', '0')
    bp_dvs.set_attribute('sigma_negative_threshold', '0')
    bp_dvs.set_attribute('refractory_period_ns', '0.0')
    bp_dvs.set_attribute('use_log', 'true')
    bp_dvs.set_attribute('log_eps', '0.001')
    camera_dvs = world.spawn_actor(
        bp_dvs,
        carla.Transform(carla.Location(x=0, y=0.2, z=2.8),
                        carla.Rotation(pitch=0, roll=0, yaw=0)),
        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

    gnss_sensor = GnssSensor(vehicle)
    imu_sensor = IMUSensor(vehicle)

    return vehicle, camera_rgb, camera_semseg, camera_depth, camera_dvs, gnss_sensor, imu_sensor


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

# ==============================================================================
# -- Weather Control -----------------------------------------------------------
# ==============================================================================


def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = (70 * math.sin(self._t)) - 20

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)


class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0

    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 85.0)
        self.wetness = clamp(self._t * 5, 0.0, 100.0)
        self.wind = 5.0 if self.clouds <= 20 else 90 if self.clouds >= 70 else 40
        self.fog = clamp(self._t - 10, 0.0, 30.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False

    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)


class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        self._storm.tick(delta_seconds)
        self.weather.cloudiness = self._storm.clouds
        self.weather.precipitation = self._storm.rain
        self.weather.precipitation_deposits = self._storm.puddles
        self.weather.wind_intensity = self._storm.wind
        self.weather.fog_density = self._storm.fog
        self.weather.wetness = self._storm.wetness
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    def __str__(self):
        return '%s %s' % (self._sun, self._storm)

# ==============================================================================
# -- Main Function--------------------------------------------------------------
# ==============================================================================


def main():
    actor_list = []
    pygame.init()
    pygame.display.set_caption(
        'IBISCape: Multi-modal Data Acquisition Framework')

    display = pygame.display.set_mode(
        (1024, 1024),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    speed_factor = 3.0  # Normal is 1.0
    update_freq = 0.1 / speed_factor
    weather = Weather(world.get_weather())
    Elapsed_time = 0.0
    iterator = 1

    try:
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        vehicle, camera_rgb, camera_semseg, camera_depth, camera_dvs, gnss_sensor, imu_sensor = create_main_actors(
            start_pose, waypoint)
        file_veh_sim = open(os.path.join(path11, '%s_simulation.txt' %
                                         get_actor_display_name(vehicle, truncate=200)), "a")
        actor_list.append(vehicle)
        actor_list.append(camera_rgb)
        actor_list.append(camera_semseg)
        actor_list.append(camera_depth)
        actor_list.append(camera_dvs)
        actor_list.append(gnss_sensor)
        actor_list.append(imu_sensor)

        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_rgb, camera_semseg, camera_depth, camera_dvs, fps=20) as sync_mode:

            while True:

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        vehicle.set_autopilot(False)
                        pygame.quit()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == K_r:
                            self.recording = not self.recording
                        elif event.key == pygame.K_ESCAPE:
                            vehicle.set_autopilot(False)
                            return

                clock.tick()

                # Advance the simulation and wait for the data.
                snapshot, image_rgb, image_semseg, image_depth, image_dvs = sync_mode.tick(
                    timeout=2.0)

                Pose = vehicle.get_transform()
                Accl = vehicle.get_acceleration()
                velo = vehicle.get_velocity()
                AngV = vehicle.get_angular_velocity()
                Ctrl = vehicle.get_control()
                t_world = snapshot.timestamp.elapsed_seconds*(10**9)

                # Update weather state
                Timestamp = snapshot.timestamp
                Elapsed_time += Timestamp.delta_seconds
                if Elapsed_time > update_freq:
                    weather.tick(speed_factor * Elapsed_time)
                    world.set_weather(weather.weather)
                    Elapsed_time = 0.0

                image_depth.convert(cc.LogarithmicDepth)
                image_semseg.convert(cc.CityScapesPalette)

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                draw_image(display, image_dvs, image_rgb, image_semseg, image_depth, t_world)

                if self.recording:
                    file_GNSS_sync.write("%d;%21.21f;%21.21f;%21.21f \n" % (
                        t_world, gnss_sensor.lat, gnss_sensor.lon, gnss_sensor.alt))
                    file_IMU_sync.write("%d;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f \n" % (
                        t_world, imu_sensor.gyroscope[0], imu_sensor.gyroscope[1], imu_sensor.gyroscope[2], imu_sensor.accelerometer[0], imu_sensor.accelerometer[1], imu_sensor.accelerometer[2]))
                    quat = euler_to_quaternion(math.radians(Pose.rotation.yaw), math.radians(Pose.rotation.pitch), math.radians(Pose.rotation.roll))
                    file_groundtruth_sync.write("%d;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f;%21.21f \n" % (t_world, Pose.location.x, Pose.location.y, Pose.location.z, velo.x, velo.y, velo.z, Accl.x, Accl.y, Accl.z, quat[0], quat[1], quat[2], quat[3], math.radians(AngV.x), math.radians(AngV.y), math.radians(AngV.z)))
                    file_veh_sim.write("%d;%f;%f;%f \n" % (
                        t_world, Ctrl.throttle, Ctrl.steer, Ctrl.brake))
                    file_cond.write('\r' + str(t_world) + ';' +
                                    str(weather) + 12 * ' ')
                    display.blit(font.render('Acquisition Frame Number: %d' % (
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
                display.blit(font.render('Accelero: (%5.8f,%5.8f,%5.8f)' % (
                    imu_sensor.accelerometer), True, (250, 0, 0)), (50, 64))
                display.blit(font.render('Gyroscop: (%5.8f,%5.8f,%5.8f)' % (
                    imu_sensor.gyroscope), True, (250, 0, 0)), (50, 82))
                display.blit(font.render('GNSS:% 24s' % ('(% 3.8f, % 3.8f, % 3.8f)' % (
                    gnss_sensor.lat, gnss_sensor.lon, gnss_sensor.alt)), True, (250, 0, 0)), (50, 100))
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

                pygame.display.flip()

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        file_rgb.close()
        file_depth.close()
        file_sseg.close()
        file_davis.close()
        file_groundtruth_sync.close()
        file_GNSS_sync.close()
        file_IMU_sync.close()
        file_events.close()
        file_veh_sim.close()
        file_cond.close()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
