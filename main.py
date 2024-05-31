from vehicle_motion_api import VehicleMotionAPI
import rospy
from std_msgs.msg import Header, String, Int32
import time

import carla
import math
from tools.crash_detection import check_if_crashed

def get_traffic_signs(world):
    # Get all actors
    actors = world.get_actors()
    traffic_signs = []
    for actor in actors:
        if 'traffic' in actor.type_id and (not 'unknown' in actor.type_id and not 'traffic_light' in actor.type_id):
            traffic_signs.append(actor)
    return traffic_signs

def get_sign_yaw(sign):
    base_transform = sign.get_transform()
    base_rot = base_transform.rotation.yaw
    return base_rot

def convert_to_carla_location(location):
    return carla.Location(x=location.x, y=location.y, z=location.z)

def quaternion_to_yaw(x, y, z, w):
    yaw = math.atan2(2.0*(w*z + x*y) , 1.0 - 2.0*(y*y + z*z))
    yaw_degrees = math.degrees(yaw)
    return yaw_degrees

def affected_by_traffic_light(vehicle_location=None, vehicle_rotation=None, signs_list=None, max_distance=None, world=None):
    # get vehicle information
    vehicle_location = convert_to_carla_location(vehicle_location)
    vehicle_yaw = vehicle_rotation.yaw

    for sign in signs_list:
        sign_location = sign.get_transform().location
        sign_yaw = sign.get_transform().rotation.yaw
        sign_location = convert_to_carla_location(sign_location)
        # sign_yaw = quaternion_to_yaw(sign.get_transform().rotation.x, sign.get_transform().rotation.y, sign.get_transform().rotation.z, sign.get_transform().rotation.w)
        distance = math.sqrt((sign_location.x - vehicle_location.x)**2 + (sign_location.y - vehicle_location.y)**2)
        angle = sign_yaw - vehicle_yaw
        if distance < max_distance and (0 < angle < 90):
            return (True, sign, distance, angle)

    return (False, None, None, None)

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

        # Filter the actors to get only the traffic signs
        signs = get_traffic_signs(world)

        VM = VehicleMotionAPI()
        time.sleep(1)
        while (not VM.General.exit_flag):
            vehicle_location = VM.General.get_vehicle_location()
            vehicle_rotation = VM.General.get_vehicle_rotation()
            affected, sign, dis, angle = affected_by_traffic_light(vehicle_location=vehicle_location, vehicle_rotation=vehicle_rotation, signs_list=signs, max_distance=50, world=world)
            if affected:
                print("Affected by traffic sign")
                print(sign.type_id)
                print("Distance: ", dis)
                print("Angle: ", angle)
            else:
                print("Not affected by traffic sign")
            
            obstacle = VM.General.get_vehicle_obstacle()
            if obstacle:
                print("Obstacle detected")
                print(obstacle)

            check_if_crashed(VM.General)
            time.sleep(0.01)
            """ End application code """
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    