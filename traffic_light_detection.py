from vehicle_motion_api import VehicleMotionAPI
import rospy
from std_msgs.msg import Header, String, Int32
import time

import carla
import math
import numpy as np

def is_within_distance(target_transform, reference_transform, max_distance, angle_interval=None):
    """
    Check if a location is both within a certain distance from a reference object.
    By using 'angle_interval', the angle between the location and reference transform
    will also be tkaen into account, being 0 a location in front and 180, one behind.

    :param target_transform: location of the target object
    :param reference_transform: location of the reference object
    :param max_distance: maximum allowed distance
    :param angle_interval: only locations between [min, max] angles will be considered. This isn't checked by default.
    :return: boolean
    """
    target_vector = np.array([
        target_transform.location.x - reference_transform.location.x,
        target_transform.location.y - reference_transform.location.y
    ])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    # Further than the max distance
    if norm_target > max_distance:
        return False

    # We don't care about the angle, nothing else to check
    if not angle_interval:
        return True

    min_angle = angle_interval[0]
    max_angle = angle_interval[1]

    fwd = reference_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return min_angle < angle < max_angle

def quaternion_to_yaw(x, y, z, w):
    yaw = math.atan2(2.0*(w*z + x*y) , 1.0 - 2.0*(y*y + z*z))
    yaw_degrees = math.degrees(yaw)
    return yaw_degrees

def get_trafficlight_trigger_location(traffic_light):
    """
    Calculates the yaw of the waypoint that represents the trigger volume of the traffic light
    """
    def rotate_point(point, radians):
        """
        rotate a given point by a given angle
        """
        rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
        rotated_y = math.sin(radians) * point.x - math.cos(radians) * point.y

        return carla.Vector3D(rotated_x, rotated_y, point.z)

    base_transform = traffic_light.get_transform()
    # base_transform = traffic_light.transform.orientation
    base_rot = base_transform.rotation.yaw
    # base_rot = quaternion_to_yaw(base_transform.x, base_transform.y, base_transform.z, base_transform.w)
    # print(base_rot)
    area_loc = base_transform.transform(traffic_light.trigger_volume.location)
    area_ext = traffic_light.trigger_volume.extent

    point = rotate_point(carla.Vector3D(0, 0, area_ext.z), math.radians(base_rot))
    point_location = area_loc + carla.Location(x=point.x, y=point.y)

    return carla.Location(point_location.x, point_location.y, point_location.z)

def convert_to_carla_location(location):
    return carla.Location(x=location.x, y=location.y, z=location.z)

def affected_by_traffic_light(lights_list=None, max_distance=None, world=None, last_traffic_light=None):
    """
    Method to check if there is a red light affecting the vehicle.

        :param lights_list (list of carla.TrafficLight): list containing TrafficLight objects.
            If None, all traffic lights in the scene are used
        :param max_distance (float): max distance for traffic lights to be considered relevant.
            If None, the base threshold value is used
    """
    actors = world.get_actors()

        # Filter the actors to get only the vehicles
    vehicles = [actor for actor in actors if 'vehicle' in actor.type_id]
    vehicle = vehicles[0]
    # print(vehicle)

    if not lights_list:
        lights_list = world.get_actors().filter("*traffic_light*")

    if not max_distance:
        max_distance = 8

    if last_traffic_light:
        if last_traffic_light.state != carla.TrafficLightState.Red:
            last_traffic_light = None
        else:
            return (True, last_traffic_light)

    ego_vehicle_location = vehicle.get_location()
    map = world.get_map()
    ego_vehicle_waypoint = map.get_waypoint(ego_vehicle_location)

    for traffic_light in lights_list:
        traffic_light = world.get_actor(traffic_light.id)
        object_location = get_trafficlight_trigger_location(traffic_light)
        object_waypoint = map.get_waypoint(object_location)

        if object_waypoint.road_id != ego_vehicle_waypoint.road_id:
            # print("Not on the same road")
            continue

        ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
        wp_dir = object_waypoint.transform.get_forward_vector()
        dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

        if dot_ve_wp < 0:
            continue

        if traffic_light.state != carla.TrafficLightState.Red:
            continue

        if is_within_distance(object_waypoint.transform, vehicle.get_transform(), max_distance, [0, 90]):
            last_traffic_light = traffic_light
            return (True, traffic_light)

    return (False, None)

def main():
    """
    Main function
    """
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2.0)

        # Get the world object from the server
        world = client.get_world()

        # Get the map of the world
        map = world.get_map()

        VM = VehicleMotionAPI()

        time.sleep(1)

        last_traffic_light = None

        while (not VM.General.exit_flag):
            """ Begin application code """

            affected, traffic_light = affected_by_traffic_light(lights_list=VM.General.get_traffic_lights(), max_distance=None, last_traffic_light=last_traffic_light, world=world)
            if affected:
                last_traffic_light = traffic_light
                print(traffic_light.id)
                VM.General.vehicle_control_manual_override(True)
                VM.General.vehicle_control_manual(throttle=0.0, steer=0.0, brake=1.0)
            else:
                last_traffic_light = None
                VM.General.vehicle_control_manual_override(False)
                print("No traffic light affecting the vehicle")

            time.sleep(0.1)
            
            """ End application code """
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    