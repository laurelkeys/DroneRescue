import airsim

import drone_orbit
import os
import time
import math
from PIL import Image


def OrbitAnimal(cx: float, cy: float, radius: float, speed: float, altitude: float, camera_angle: float, animal: str):
    '''
    @param cx: The x position of our orbit starting location\n
    @param cy: The y position of our orbit starting location\n
    @param radius: The radius of the orbit circle\n
    @param speed: The speed the drone should more, it's hard to take photos when flying fast\n
    @param altitude: The altidude we want to fly at, dont fly too high!\n
    @param camera_angle: The angle of the camera\n
    @param animal: The name of the animal, used to prefix the photos
    '''

    x = cx - radius
    y = cy

    # set camera angle
    client.simSetCameraOrientation(camera_name="0", orientation=airsim.to_quaternion(math.radians(camera_angle), 0, 0))

    # move the drone to the requested location
    print("moving to position...")
    client.moveToPositionAsync(x, y, z, velocity=1, timeout_sec=60, 
                               drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, 
                               yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
    pos = client.getMultirotorState().kinematics_estimated.position

    dx = x - pos.x_val
    dy = y - pos.y_val
    _roll, _pitch, yaw = airsim.to_eularian_angles(client.getMultirotorState().kinematics_estimated.orientation)

    # keep the drone on target, it's windy out there!
    print("correcting position and yaw...")
    while abs(dx) > 1 or abs(dy) > 1 or abs(yaw) > 0.1:
        client.moveToPositionAsync(x, y, z, velocity=0.25, timeout_sec=60, 
                                   drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, 
                                   yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
        pos = client.getMultirotorState().kinematics_estimated.position
        dx = x - pos.x_val
        dy = y - pos.y_val
        _roll, _pitch, yaw = airsim.to_eularian_angles(client.getMultirotorState().kinematics_estimated.orientation)
        print("yaw is {}".format(yaw))

    print("location is off by {},{}".format(dx, dy))

    o = airsim.to_eularian_angles(client.getMultirotorState().kinematics_estimated.orientation)
    print("yaw is {}".format(o[2]))

    # let's orbit around the animal and take some photos
    nav = drone_orbit.OrbitNavigator(radius=radius, altitude=altitude, speed=speed, 
                                     iterations=1, center=[cx - pos.x_val, cy - pos.y_val], snapshots=30, 
                                     photo_prefix=animal, image_dir=os.path.join(".", "drone_images"))
    nav.start()


def land():
    print("landing...")
    client.landAsync().join()

    print("disarming.")
    client.armDisarm(False)

    client.reset()
    client.enableApiControl(False)

def orbit():
    OrbitAnimal(19.6, 9.6, 2, 0.4, 1, -30, "BlackSheep")
    #OrbitAnimal(-12.18, -13.56, 2, 0.4, 1, -30, "AlpacaRainbow")
    #OrbitAnimal(-12.18, -13.56, 3, 0.4, 1, -20, "AlpacaRainbow")

    #animals = [( 19.80, -11.00, "AlpacaPink"),
    #           (  5.42,  -3.70, "AlpacaTeal"),
    #           (-12.18, -13.56, "AlpacaRainbow"),
    #           ( 19.60,   9.60, "BlackSheep"),
    #           ( -1.90,  -0.90, "Bunny"),
    #           (  3.50,   9.40, "Chick"),
    #           (-13.20,  -0.25, "Chipmunk"),
    #           ( -6.55,  12.25, "Hippo")]
    #configurations = [(2, 0.4, 1, -30), (3, 0.4, 1, -20)]
    ## let's find the animals and take some photos
    #for config in configurations:
    #   for animal in animals:
    #       print("Target animal:" + str(animal[2]))
    #       radius = config[0]
    #       speed = config[1]
    #       camera_angle = config[2]
    #       OrbitAnimal(animal[0], animal[1], radius, speed, 1, camera_angle, animal[2])

    #OrbitAnimal(15, 1.0, 2, 0.4, 1, -30, "Unicorn")

if __name__ == '__main__':
    # Conect with the airsim server
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    # Check State and takeoff if required
    landed = client.getMultirotorState().landed_state

    if landed == airsim.LandedState.Landed:
        print("taking off...")
        pos = client.getMultirotorState().kinematics_estimated.position
        z = pos.z_val - 1
        client.takeoffAsync().join()
    else:
        print("already flying...")
        client.hover()
        pos = client.getMultirotorState().kinematics_estimated.position
        z = pos.z_val

    # Start the navigation task
    orbit()

    # that's enough fun for now, let's quit cleanly
    land()

    print("Image capture complete...")
