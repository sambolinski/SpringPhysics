# Spring Physics Simulation
![img](https://i.imgur.com/dguoYyk.gif)
This project is a 2D pixel simulation of spring connections between nodes using C++ and the [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine). The original goal was to simulate rope, which was achieved by connecting many nodes closely together. Properties such as tension and stretching can be shown, furthermore, if a few nodes are connected it also simulates soft-body physics.

# Features
- Physics
  - Spring physics based off equillebroium distance between nodes.
  - Force application such as gravity.
  - Toggle air resistance
- Interaction
  - Delect node to lock in position
  - Add to nodes and connect nodes together
  - Delete nodes 
- Graphics
  - Nodes can be drawn as circles or not drawn
  - Springs drawn as lines
  - GUI to show details such as number of nodes, whether the user is in edit mode.


- Controls
- Left Click: Grab node
- Right Click: Lock node
- Space: Toggle pause
- A: Toggle air resistance 
- B: Toggle rendering nodes
- Mouse Wheel Up/Down: Add to end of string / Remove from end of string
- E: Edit Mode
  - While in edit mode
    - Left Click: Select node
    - Right Click: Add node 
