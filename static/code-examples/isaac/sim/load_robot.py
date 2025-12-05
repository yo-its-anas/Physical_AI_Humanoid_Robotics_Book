#!/usr/bin/env python3
"""
Load Robot in NVIDIA Isaac Sim

This script demonstrates loading a USD robot model into Isaac Sim.

Isaac Sim Version: 2023.1+
Dependencies: omni.isaac.kit, omni.usd
"""

from omni.isaac.kit import SimulationApp

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf

def main():
    """
    Main function to load robot into Isaac Sim scene.
    """
    # Create world
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    world.scene.add_default_ground_plane()

    # Load robot USD file
    robot_prim_path = "/World/Robot"
    robot_usd_path = "/path/to/robot.usd"  # Update with actual path

    add_reference_to_stage(
        usd_path=robot_usd_path,
        prim_path=robot_prim_path
    )

    # Set robot position
    robot_prim = world.stage.GetPrimAtPath(robot_prim_path)
    robot_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, 0.5))

    print(f"Robot loaded at: {robot_prim_path}")

    # Reset and run simulation
    world.reset()

    # Simulation loop
    for i in range(1000):
        world.step(render=True)

        if i % 100 == 0:
            print(f"Simulation step: {i}")

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
