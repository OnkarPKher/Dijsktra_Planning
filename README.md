# Pathfinding Visualization

This project implements a pathfinding algorithm to visualize the optimal path generation between a start point and a goal point on a grid map containing various obstacles. The visualization showcases both node exploration and the generated optimal path.

## How to Run the Code

To run this code, ensure you have Python installed on your system. Follow these steps:

1. **Install Dependencies**: The project requires the following Python libraries:
   - numpy
   - OpenCV-Python

   You can install these dependencies using pip:

   **Running the Script**: Open a terminal or command prompt, navigate to the project directory, and run:
   
`   python dijsktra_onkar_kher.py`

   **Inputting Start and Goal Coordinates**: The script will prompt you to enter the x and y coordinates for both the start node and the goal node. Enter these as integer values when prompted. Assume that the coordinate system origin (0,0) is located at the bottom left of the grid and enter the coordinates accordingly. 

Example input for start node: 

Please enter the x coordinate of the start node of the point robot, x1: 0

Please enter the y coordinate of the start node of the point robot, y1: 100

And similarly for the goal node.

## Libraries/Dependencies

- **numpy**: Used for numerical operations and to manage the grid representation.
- **OpenCV-Python (cv2)**: Utilized for visualizing the grid, obstacles, node exploration, and the optimal path.

Make sure all dependencies are installed to ensure the script runs successfully.



