import carla    
import time
import math 
import numpy as np  # type: ignore
from collections import deque 
from agents.navigation import controller
from rdp import rdp # type: ignore

# Initialize the actor with a specific spawn point
def init_actor(client):
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    
    # Verify blueprint exists
    vehicle_bp = blueprint_library.find("vehicle.tesla.cybertruck")
    if not vehicle_bp:
        print("Blueprint for vehicle not found.")
        return None
    
    # Set specific spawn coordinates directly
    spawn_transform = carla.Transform(
        location=carla.Location(x=-4.29, y=-124.02, z=0.5),  # Adjust z as needed for height above the ground
        rotation=carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0)  # Adjust yaw to face the correct direction
    )
    
    # Attempt to spawn the actor
    actor = world.try_spawn_actor(vehicle_bp, spawn_transform)
    if actor is None:
        print("Failed to spawn actor.")
        return None
    
    return actor

def setup_spectator_camera_once(world, actor):
    spectator = world.get_spectator()
    actor_transform = actor.get_transform()
    
    # Position the spectator camera at a fixed offset behind and above the actor
    offset = carla.Location(x=-10.0, z=5.0)  # Adjust for distance and height
    spectator.set_transform(carla.Transform(
        actor_transform.location + offset,
        actor_transform.rotation
    ))
# Initialize the Vehicle PID Controller
def init_vehicle_pid(actor):
    args_lateral = {
        "K_P": 0.6,  # Proportional term for lateral control
        "K_D": 0.81,  # Differential term for lateral control
        "K_I": 0.55  # Integral term for lateral control
    }

    args_longitudinal = {
        "K_P": 1.0,  # Proportional term for longitudinal control
        "K_D": 0.05,  # Differential term for longitudinal control
        "K_I": 0.7   # Integral term for longitudinal control
    }

    # Initialize the controller with the actor and PID settings
    vehicle_pid = controller.VehiclePIDController(
        actor, 
        args_lateral=args_lateral, 
        args_longitudinal=args_longitudinal, 
        offset=0,
        max_throttle=1.0, 
        max_steering=0.8
    )
    
    return vehicle_pid

# Initialize the world and set the map
def init_world(client, map_name="Town03"):
    # Load the specified map
    world = client.load_world(map_name)
    
    settings = world.get_settings()
    settings.synchronous_mode = False # Asynchronous mode for stability
    settings.fixed_delta_seconds = 0.0  # Lower frame rate to reduce load
    settings.tile_stream_distance = 1000  # Lower streaming distance
    settings.actor_active_distance = 1000  # Lower active distance
    world.apply_settings(settings)
    
    return world
    
def load_waypoints_from_file(carla_map, filename="waypoints.txt"):
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split())
            location = carla.Location(x=x, y=-y, z=0.0)  # Assuming z = 0 for 2D path
            waypoint = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
            print("Loading Waypoints ...")
            if waypoint:
                waypoints.append(waypoint.transform.location)  # Store only locations for RDP
            else:
                print(f"No waypoint found at location ({x}, {y}).")
    
    # Simplify waypoints with RDP, using an epsilon for tolerance
    simplified_waypoints = rdp(np.array([[wp.x, wp.y] for wp in waypoints]), epsilon=0.7)
    
    # Convert the simplified waypoint coordinates back to CARLA waypoints
    simplified_carla_waypoints = [
        carla_map.get_waypoint(carla.Location(x=wp[0], y=wp[1], z=0.0)) for wp in simplified_waypoints
    ]
    
    return simplified_carla_waypoints
# Main function to initialize CARLA, load waypoints, and control the vehicle
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Initialize world and map
    world = init_world(client, "Town03")
    carla_map = world.get_map()
    
    # Initialize vehicle and PID controller
    actor = init_actor(client)
    if actor is None:
        print("Exiting: actor could not be spawned.")
        return
    
    actor_id = actor.id
    print(f"Actor id = {actor_id}")

    vehicle_pid = init_vehicle_pid(actor)
    setup_spectator_camera_once(world, actor) 
    # Load waypoints
    waypoints = load_waypoints_from_file(carla_map, "waypoints.txt")
    if not waypoints:
        print("No waypoints loaded. Exiting.")
        return

    # Control loop
    print("Controlling actor")


    output_file = "actor_path.txt"
    last_saved_location = None 


    for waypoint in waypoints:

        world_snapshot = world.get_snapshot() 
        actor_snapshot = world_snapshot.find(actor_id)

        current_transform = actor_snapshot.get_transform()
        current_location = current_transform.location

        waypoint_location = waypoint.transform.location

        distance = current_location.distance(waypoint_location)


        while distance > 8:
            world_snapshot = world.get_snapshot() 
            actor_snapshot = world_snapshot.find(actor_id)

            current_transform = actor_snapshot.get_transform()
            current_location = current_transform.location
            
            # Save location if it differs significantly from the last saved one
            if last_saved_location is None or current_location.distance(last_saved_location) > 1.0:
                with open(output_file, "a") as f:
                    f.write(f"{current_location.x} {current_location.y}\n")
                last_saved_location = current_location  # Update last saved location


            waypoint_location = waypoint.transform.location

            distance = current_location.distance(waypoint_location)
            print("Distance : "+ str(distance))
            control = vehicle_pid.run_step(target_speed=50, waypoint=waypoint)
            actor.apply_control(control)
        world.tick()  # Update the world for the next simulation step
        


if __name__ == "__main__":
    main()
