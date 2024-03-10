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

After executing the code, the script will take some time to launch the "Exploration and Path" visualization window, before which it will ask you to input the start node coordinates of the point robot and also the goal node coordinates of the point robot, one at a time. The example of the prompt at the terminal for the user is given above. After the coordinates have been entered theprogram checks whether these are possible to achieve given the obstacles placed as per the project requirement in the custom environment. After all these tasks have been computed, the "Exploration and Path" window which makes the users visualize the nodes explored during the process of finding the optimal path based on the search strategy of the "Dijsktra's Algorithm". After a sufficient amount of nodes have been explored the program then displays another visualization window namely "Path-finding Visualization", which has the optimal path on it, marked with red color. The obstacles are black in color, with the clearance required (5mm) displayed in grey color. The free space is being displayed in white color. The total number of nodes explored will be displayed using green color to mark the nodes that have been explored. 

After the optimal path has been shown on the respective visualization window the terminal also prints out the path taken (optimal path) by the point robot in the user's coordinate system (origin at the bottom left corner), and also prints out the time taken to get and animate that path. To exit the program close both the visualization windows press the exit commands based on your IDE or Code editor software. (Usually `ctrl. + z` or `ctrl. + c`)

