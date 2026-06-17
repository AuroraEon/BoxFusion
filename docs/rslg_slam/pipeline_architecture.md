# RSLG-SLAM Pipeline Architecture

The conceptual system chain is:

`RGB-D + pose -> Layer 0: Input Layer -> Layer 1: World Model Layer -> Layer 2: Formal Artifact Layer -> Layer 3: Navigation Interface Layer -> Layer 4: Runtime Validation Layer`

Layer 1 is the World Model Layer. Do not call it the "Stage-A Layer". Stage-A is only a historical implementation entrypoint, for example `stage_a_demo.py`.

## Layers

### Layer 0: Input Layer

Normalizes and records scene inputs: RGB images, depth images, and provided camera poses.

### Layer 1: World Model Layer

Builds the rich-semantic, light-geometry indoor world model. It produces semantic, topological, connector, object, and floor/room structure needed by downstream builders.

### Layer 2: Formal Artifact Layer

Exports formal route, connector, object-interface, stable occupancy map, and provenance artifacts from the world model. Stable occupancy map generation belongs here: it is a floor-wise navigation raster generated from World Model Layer BEV/free-space/wall/outside-boundary/gateway evidence.

### Layer 3: Navigation Interface Layer

Builds routing and execution-facing interfaces from formal artifacts. Stable occupancy map consumption belongs here for A*, map_server, lightweight executor, and validation-facing navigation interfaces.

### Layer 4: Runtime Validation Layer

Runs validation or demonstrations against formal artifacts and navigation interfaces. Runtime logs, costmaps, and process traces are evidence, not source world-model artifacts.

## Python Environment Contract

- World Model / offline Python: `/home/ws/miniconda3/envs/boxfusion/bin/python`
- ROS2 / Gazebo / RViz / rclpy runtime Python: `/usr/bin/python3`

This task did not rerun the World Model Layer, Stage-A, Gazebo, RViz, Nav2, AMCL, or runtime demos.

## Map Distinctions

- `gateway_wall_preclose`: used for gateway/topology extraction.
- `segmentation_wall_processed`: used for room segmentation provenance.
- Clean floorplan / BEV: used for visualization.
- Stable occupancy map: Layer 2 floor-wise navigation raster generated from World Model Layer BEV/free-space/wall/outside-boundary/gateway evidence and consumed by Layer 3 navigation interfaces. It is not an external map, semantic floorplan, room mask, or runtime costmap.
- Local/global costmap: runtime internal state, not an RSLG-SLAM world-model artifact.
