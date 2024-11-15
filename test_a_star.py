from a_star import A_Star_Search
import numpy as np

# test for the A-star class
def main():
    # Define the grid (1 for unblocked, 0 for blocked)
    grid = np.array([
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
    ])

    # Define the source and destination
    src = [0, 0]
    dest = [8, 9]

    searchalg = A_Star_Search(grid, src, dest)
    path = searchalg.search()
    if path == []:
        print("Failed to find the destination cell")
    else:
        print("The destination cell is found")
        print("The Path is ")
        print(path)

if __name__ == "__main__":
    main()