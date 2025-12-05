#!/usr/bin/env python3
"""
Synthetic Data Generation in Isaac Sim

Generate synthetic training datasets with domain randomization.

Isaac Sim Version: 2023.1+
Dependencies: omni.replicator
"""

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from omni.isaac.core import World

def main():
    """
    Generate synthetic dataset with randomized scenes.
    """
    # Create world
    world = World()

    # Setup camera
    camera = rep.create.camera(
        position=(2, 2, 2),
        look_at=(0, 0, 0)
    )

    # Setup render products
    render_product = rep.create.render_product(
        camera,
        resolution=(512, 512)
    )

    # Annotators for ground truth data
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    bbox_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")

    rgb_annot.attach([render_product])
    depth_annot.attach([render_product])
    bbox_annot.attach([render_product])

    # Randomization function
    def randomize_scene():
        # Randomize lighting
        light = rep.get.light()
        with light:
            rep.modify.attribute("intensity", rep.distribution.uniform(500, 1500))
            rep.modify.attribute("color", rep.distribution.uniform((0.5, 0.5, 0.5), (1, 1, 1)))

        # Randomize object poses
        objects = rep.get.prims(path_pattern="/World/Objects/*")
        with objects:
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

    # Register randomization
    with rep.trigger.on_frame(num_frames=100):
        randomize_scene()

    # Run data generation
    print("Generating synthetic data...")
    rep.orchestrator.run()

    # Save data
    print("Data generation complete. Check output directory.")

    simulation_app.close()

if __name__ == "__main__":
    main()
