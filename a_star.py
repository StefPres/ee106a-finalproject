# Python program for A* Search Algorithm
# Adapted from: https://www.geeksforgeeks.org/a-search-algorithm-in-python/
import numpy as np
import math
import heapq

# Define the Cell class
class Cell:
    def __init__(self):
      # Parent cell's row index
        self.parent_i = 0
    # Parent cell's column index
        self.parent_j = 0
    # Total cost of the cell (g + h)
        self.f = float('inf')
    # Cost from start to this cell
        self.g = float('inf')
    # Heuristic cost from this cell to destination
        self.h = 0

class A_Star_Search:
    def __init__(self, grid, start, dest):
        self.grid = grid #numpy array. 0 = blocked, 1 = unblocked
        self.src = start #point
        self.dest = dest #point
    
    # prevent needing to create a new object if stuff changes
    def update_start(self, start):
        if self.is_valid(start):
            self.src = start
        else:
            print("Source point is invalid")
    
    def update_dest(self, dest):
        if self.is_valid(dest):
            self.dest = dest
        else:
            print("Destination point is invalid")

    def update_grid(self,grid):
        self.grid = grid
    
    # utility functions
    def is_valid(self, point):
        return (point[0] >= 0) and (point[0] < self.grid.shape[0]) and (point[1] >= 0) and (point[1] < self.grid.shape[1])
    
    def is_unblocked(self, point):
        return self.grid[point[0],point[1]] == 1
    
    def is_destination(self, point):
        return point[0] == self.dest[0] and point[1] == self.dest[1]
    
    def calculate_h_value(self, point):
        return ((point[0] - self.dest[0]) ** 2 + (point[1] - self.dest[1]) ** 2) ** 0.5
    
    def trace_path(self, cell_details):
        path = []
        row = self.dest[0]
        col = self.dest[1]

        # Trace the path from destination to source using parent cells
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        # Add the source cell to the path
        path.append((row, col))
        # Reverse the path to get the path from source to destination
        path.reverse()

        # Print the path
        return path

    def search(self):
        # Check if the source and destination are valid
        if not self.is_valid(self.src) or not self.is_valid(self.dest):
            print("Source or destination is invalid")
            return []

        # Check if the source and destination are unblocked
        if not self.is_unblocked(self.src) or not self.is_unblocked(self.dest):
            print("Source or the destination is blocked")
            return []

        # Check if we are already at the destination
        if self.is_destination(self.src):
            print("We are already at the destination")
            return []

        # Initialize the closed list (visited cells)
        closed_list = [[False for _ in range(self.grid.shape[1])] for _ in range(self.grid.shape[0])]
        # Initialize the details of each cell
        cell_details = [[Cell() for _ in range(self.grid.shape[1])] for _ in range(self.grid.shape[0])]

        # Initialize the start cell details
        i = self.src[0]
        j = self.src[1]
        cell_details[i][j].f = 0
        cell_details[i][j].g = 0
        cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        # Initialize the open list (cells to be visited) with the start cell
        open_list = []
        heapq.heappush(open_list, (0.0, i, j))

        # Initialize the flag for whether destination is found
        found_dest = False

        # Main loop of A* search algorithm
        while len(open_list) > 0:
            # Pop the cell with the smallest f value from the open list
            p = heapq.heappop(open_list)

            # Mark the cell as visited
            i = p[1]
            j = p[2]
            closed_list[i][j] = True

            # For each direction, check the successors
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                        (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                # If the successor is valid, unblocked, and not visited
                if self.is_valid([new_i, new_j]) and self.is_unblocked([new_i, new_j]) and not closed_list[new_i][new_j]:
                    # If the successor is the destination
                    if self.is_destination([new_i,new_j]):
                        # Set the parent of the destination cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        # Trace and print the path from source to destination
                        foundpath = self.trace_path(cell_details)
                        found_dest = True
                        return foundpath
                    else:
                        # Calculate the new f, g, and h values
                        g_new = cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value([new_i, new_j])
                        f_new = g_new + h_new

                        # If the cell is not in the open list or the new f value is smaller
                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            # Add the cell to the open list
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            # Update the cell details
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        # If the destination is not found after visiting all cells
        if not found_dest:
            return []
