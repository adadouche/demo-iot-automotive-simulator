#!/usr/bin/env python

"""
CARLA CAN BUS Vehicle Integration
"""

import argparse
import logging
import time
import math

import can
import carla
import cantools
import weakref


class CarlaCan:
    def __init__(self, database, interface):
        logging.info('loading %s dbc database', database)
        self.db = cantools.database.load_file(database)
        self.can_bus = can.interface.Bus(interface, bustype='socketcan')

        self.msg_speed = self.db.get_message_by_name('WheelSpeeds')
        self.msg_control = self.db.get_message_by_name('Control')
        self.msg_steering = self.db.get_message_by_name('SteeringWheel')
        self.msg_location = self.db.get_message_by_name('Location')

    def send_speed(self, v):
        speed = (3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))
        data = self.msg_speed.encode({
            'Wheel1': speed,
            'Wheel2': speed,
            'Wheel3': speed,
            'Wheel4': speed,
        })
        message = can.Message(arbitration_id=self.msg_speed.frame_id, data=data)
        self.can_bus.send(message)

    def send_control(self, c):
        data = self.msg_control.encode({
            'Throttle': c.throttle,
            'Brake': c.brake,
            'Gear': c.gear,
            'Reverse': c.reverse,
            'HandBrake': c.hand_brake,
            'ManualGearShift': c.manual_gear_shift,
        })
        message = can.Message(arbitration_id=self.msg_speed.frame_id, data=data)
        self.can_bus.send(message)

    def send_steering(self, steering):
        data = self.msg_steering.encode({
            'SteeringPosition': steering
        })
        message = can.Message(arbitration_id=self.msg_speed.frame_id, data=data)
        self.can_bus.send(message)

    def send_location(self, t, imu_sensor):
        data = self.msg_location.encode({
            'LocationX': t.location.x,
            'LocationY': t.location.y,
            'Compass': imu_sensor.compass,
        })
        message = can.Message(arbitration_id=self.msg_speed.frame_id, data=data)
        self.can_bus.send(message)


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor( bp, carla.Transform(), attach_to=self._parent)

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
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


def main_loop(args):
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    # Set synchronous/asynchronous mode settings
    if args.sync:
        settings = world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Set up the traffic manager
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

    # Find the Vehicle
    for vehicle in world.get_actors().filter(args.filter):
        if isinstance(vehicle, carla.Vehicle):
            player = vehicle
            break

    # IMU Sersor attach
    imu_sensor = IMUSensor(player)

    # CAN BUS
    can_bus = CarlaCan(args.database, args.interface)

    # Simulation loop
    while True:
        if args.sync:
            world.tick()
        else:
            world.wait_for_tick()

        can_bus.send_speed(player.get_velocity())
        can_bus.send_control(player.get_control())
        can_bus.send_steering(player.get_control().steer * 600)
        can_bus.send_location(player.get_transform(), imu_sensor)
        time.sleep(0.5)


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA CAN BUS Vehicle Integration')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-i', '--interface',
        default='vcan0',
        help='CAN interface, e.g. vcan0'
    )
    argparser.add_argument(
        '-d',
        '--database',
        required=True,
        help='DBC file'
    )
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    print(__doc__)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:
        main_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
