import sys
import math
from collections import deque
import heapq

STUDENT_ID = 'a1878646'
DEGREE = 'UG'

# Constants for map parsing and grid representation
MAP_OBSTACLE_CHARACTER = 'X'
MAP_OBSTACLE_VALUE = -1

# Defining movement directions as constants for clarity and configurability
DIRECTION_UPWARD = (-1, 0)
DIRECTION_DOWNWARD = (1, 0)
DIRECTION_LEFTWARD = (0, -1)
DIRECTION_RIGHTWARD = (0, 1)
ALL_POSSIBLE_DIRECTIONS = [
    DIRECTION_UPWARD,
    DIRECTION_DOWNWARD,
    DIRECTION_LEFTWARD,
    DIRECTION_RIGHTWARD
]

class Node:
    def __init__(self, state: tuple[int, int], parent: 'Node', g: int):
        self.state = state  # Current position in the grid
        self.parent = parent  # Parent node for path reconstruction
        self.g = g  # Accumulated cost from the start

def read_map_data_from_input_file(map_file_path):
    with open(map_file_path, 'r') as file_handle:
        # Reading the dimensions of the grid
        first_line = file_handle.readline().strip().split()
        number_of_rows = int(first_line[0])
        number_of_columns = int(first_line[1])

        # Reading start position
        second_line = file_handle.readline().strip().split()
        start_position = (int(second_line[0]), int(second_line[1]))

        # Reading end position
        third_line = file_handle.readline().strip().split()
        end_position = (int(third_line[0]), int(third_line[1]))

        # Reading grid map
        grid_map = []
        for _ in range(number_of_rows):
            line = file_handle.readline().strip().split()
            grid_map.append(line)

    return number_of_rows, number_of_columns, start_position, end_position, grid_map


def is_position_valid_within_grid(
    row_index: int,
    column_index: int,
    number_of_rows: int,
    number_of_columns: int,
    grid_map: list[list[int]]
) -> bool:
    is_row_within_bounds = 0 <= row_index < number_of_rows
    is_column_within_bounds = 0 <= column_index < number_of_columns
    if is_row_within_bounds and is_column_within_bounds:
        is_not_obstacle = grid_map[row_index][column_index] != MAP_OBSTACLE_VALUE
    else:
        is_not_obstacle = False
    position_is_valid = is_row_within_bounds and is_column_within_bounds and is_not_obstacle
    return position_is_valid

def calculate_movement_cost(
    grid_map: list[list[int]],
    from_row: int,
    from_col: int,
    to_row: int,
    to_col: int
) -> int:
    height_at_start = grid_map[from_row][from_col]
    height_at_end = grid_map[to_row][to_col]
    height_difference = height_at_end - height_at_start
    if height_difference > 0:
        total_cost = 1 + height_difference
    else:
        total_cost = 1
    return total_cost

def heuristic_distance(
    current_position: tuple[int, int],
    goal_position: tuple[int, int],
    heuristic_type: str
) -> float:
    row_difference = current_position[0] - goal_position[0]
    col_difference = current_position[1] - goal_position[1]
    if heuristic_type == 'euclidean':
        distance = math.sqrt(row_difference ** 2 + col_difference ** 2)
    elif heuristic_type == 'manhattan':
        distance = abs(row_difference) + abs(col_difference)
    else:
        raise ValueError(f"Unknown heuristic type: {heuristic_type}")
    return distance

def get_valid_neighbors(
    current_position: tuple[int, int],
    number_of_rows: int,
    number_of_columns: int,
    grid_map: list[list[int]]
) -> list[tuple[int, int]]:
    current_row, current_col = current_position
    valid_neighbors = []
    for delta_row, delta_col in ALL_POSSIBLE_DIRECTIONS:
        new_row = current_row + delta_row
        new_col = current_col + delta_col
        if is_position_valid_within_grid(new_row, new_col, number_of_rows, number_of_columns, grid_map):
            valid_neighbors.append((new_row, new_col))
    return valid_neighbors

def reconstruct_path_from_goal_node(goal_node: Node) -> list[tuple[int, int]]:
    path_list = []
    current_node = goal_node
    while current_node is not None:
        path_list.append(current_node.state)
        current_node = current_node.parent
    # Reverse the list to get the path from start to goal
    path_list.reverse()
    return path_list

def bfs_search_algorithm(
    grid_map: list[list[int]],
    start_position: tuple[int, int],
    goal_position: tuple[int, int],
    number_of_rows: int,
    number_of_columns: int,
    visit_count_matrix: list[list[int]],
    first_visit_matrix: list[list[int]],
    last_visit_matrix: list[list[int]]
) -> Node:
    fringe = deque([Node(start_position, None, 0)])
    closed_set = [[False] * number_of_columns for _ in range(number_of_rows)]
    step_counter = 0

    while fringe:
        current_node = fringe.popleft()
        step_counter += 1
        current_row, current_col = current_node.state

        # Update tracking matrices for debugging
        if first_visit_matrix[current_row][current_col] is None:
            first_visit_matrix[current_row][current_col] = step_counter
        last_visit_matrix[current_row][current_col] = step_counter
        visit_count_matrix[current_row][current_col] += 1

        if current_node.state == goal_position:
            return current_node

        if not closed_set[current_row][current_col]:
            closed_set[current_row][current_col] = True
            valid_neighbors = get_valid_neighbors(current_node.state, number_of_rows, number_of_columns, grid_map)
            for neighbor_position in valid_neighbors:
                neighbor_row, neighbor_col = neighbor_position
                movement_cost = calculate_movement_cost(
                    grid_map, current_row, current_col, neighbor_row, neighbor_col
                )
                new_node = Node(neighbor_position, current_node, current_node.g + movement_cost)
                fringe.append(new_node)
                # Commented-out debug statement for tracing
                # print(f"BFS: Added neighbor {neighbor_position} with g={new_node.g}")
    return None

def ucs_search_algorithm(
    grid_map: list[list[int]],
    start_position: tuple[int, int],
    goal_position: tuple[int, int],
    number_of_rows: int,
    number_of_columns: int,
    visit_count_matrix: list[list[int]],
    first_visit_matrix: list[list[int]],
    last_visit_matrix: list[list[int]]
) -> Node:
    fringe = [(0, 0, Node(start_position, None, 0))]  # (g_cost, counter, node)
    heapq.heapify(fringe)
    closed_set = [[False] * number_of_columns for _ in range(number_of_rows)]
    step_counter = 0
    counter = 0

    while fringe:
        _, _, current_node = heapq.heappop(fringe)
        step_counter += 1
        current_row, current_col = current_node.state

        # Update tracking matrices
        if first_visit_matrix[current_row][current_col] is None:
            first_visit_matrix[current_row][current_col] = step_counter
        last_visit_matrix[current_row][current_col] = step_counter
        visit_count_matrix[current_row][current_col] += 1

        if current_node.state == goal_position:
            return current_node

        if not closed_set[current_row][current_col]:
            closed_set[current_row][current_col] = True
            valid_neighbors = get_valid_neighbors(current_node.state, number_of_rows, number_of_columns, grid_map)
            for neighbor_position in valid_neighbors:
                neighbor_row, neighbor_col = neighbor_position
                movement_cost = calculate_movement_cost(
                    grid_map, current_row, current_col, neighbor_row, neighbor_col
                )
                new_g_cost = current_node.g + movement_cost
                counter += 1
                heapq.heappush(fringe, (new_g_cost, counter, Node(neighbor_position, current_node, new_g_cost)))
                # Commented-out debug statement
                # print(f"UCS: Added neighbor {neighbor_position} with g={new_g_cost}")
    return None

def astar_search_algorithm(
    grid_map: list[list[int]],
    start_position: tuple[int, int],
    goal_position: tuple[int, int],
    number_of_rows: int,
    number_of_columns: int,
    visit_count_matrix: list[list[int]],
    first_visit_matrix: list[list[int]],
    last_visit_matrix: list[list[int]],
    heuristic_type: str
) -> Node:
    start_node = Node(start_position, None, 0)
    initial_f_cost = heuristic_distance(start_position, goal_position, heuristic_type)
    fringe = [(initial_f_cost, 0, start_node)]  # (f_cost, counter, node)
    heapq.heapify(fringe)
    closed_set = [[False] * number_of_columns for _ in range(number_of_rows)]
    step_counter = 0
    counter = 0

    while fringe:
        _, _, current_node = heapq.heappop(fringe)
        step_counter += 1
        current_row, current_col = current_node.state

        # Update tracking matrices
        if first_visit_matrix[current_row][current_col] is None:
            first_visit_matrix[current_row][current_col] = step_counter
        last_visit_matrix[current_row][current_col] = step_counter
        visit_count_matrix[current_row][current_col] += 1

        if current_node.state == goal_position:
            return current_node

        if not closed_set[current_row][current_col]:
            closed_set[current_row][current_col] = True
            valid_neighbors = get_valid_neighbors(current_node.state, number_of_rows, number_of_columns, grid_map)
            for neighbor_position in valid_neighbors:
                neighbor_row, neighbor_col = neighbor_position
                movement_cost = calculate_movement_cost(
                    grid_map, current_row, current_col, neighbor_row, neighbor_col
                )
                new_g_cost = current_node.g + movement_cost
                heuristic_cost = heuristic_distance(neighbor_position, goal_position, heuristic_type)
                total_f_cost = new_g_cost + heuristic_cost
                counter += 1
                heapq.heappush(fringe, (total_f_cost, counter, Node(neighbor_position, current_node, new_g_cost)))
                # Commented-out debug statement
                # print(f"A*: Added neighbor {neighbor_position} with f={total_f_cost}, g={new_g_cost}, h={heuristic_cost}")
    return None

def print_matrix_with_space_separation(matrix: list[list[str]], num_rows: int, num_cols: int) -> None:
    for row_idx in range(num_rows):
        row_elements = [str(matrix[row_idx][col_idx]) for col_idx in range(num_cols)]
        print(' '.join(row_elements))

def print_grid_with_path(
    grid_map: list[list[int]],
    path: list[tuple[int, int]] = None
) -> None:
    path_set = set(path) if path else set()
    for row_idx, row in enumerate(grid_map):
        for col_idx, cell in enumerate(row):
            if (row_idx, col_idx) in path_set:
                print('* ', end='')
            elif cell == MAP_OBSTACLE_VALUE:
                print(MAP_OBSTACLE_CHARACTER + ' ', end='')
            else:
                print(str(cell) + ' ', end='')
        print()

def main():
    """Handle command-line arguments and execute the appropriate search algorithm."""
    if len(sys.argv) < 4:
        print("Usage: python pathfinder.py [mode] [map] [algorithm] [heuristic]")
        print("  mode: 'debug' or 'release'")
        print("  map: Path to the map file")
        print("  algorithm: 'bfs', 'ucs', or 'astar'")
        print("  heuristic: 'euclidean' or 'manhattan' (required for A*)")
        sys.exit(1)

    mode = sys.argv[1]
    map_file_path = sys.argv[2]
    algorithm = sys.argv[3]
    heuristic_type = sys.argv[4] if len(sys.argv) > 4 else None

    # Load map data
    number_of_rows, number_of_columns, start_position, end_position, grid_map = read_map_data_from_input_file(map_file_path)

    # Validate start and end positions
    if not is_position_valid_within_grid(start_position[0], start_position[1], number_of_rows, number_of_columns, grid_map):
        print("Error: Start position is invalid or an obstacle.")
        sys.exit(1)
    if not is_position_valid_within_grid(end_position[0], end_position[1], number_of_rows, number_of_columns, grid_map):
        print("Error: End position is invalid or an obstacle.")
        sys.exit(1)

    # Initialize matrices for tracking visits (used in debug mode)
    visit_count_matrix = [[0] * number_of_columns for _ in range(number_of_rows)]
    first_visit_matrix = [[None] * number_of_columns for _ in range(number_of_rows)]
    last_visit_matrix = [[None] * number_of_columns for _ in range(number_of_rows)]

    # Execute the selected search algorithm
    solution_node = None
    if algorithm == 'bfs':
        solution_node = bfs_search_algorithm(
            grid_map, start_position, end_position, number_of_rows, number_of_columns,
            visit_count_matrix, first_visit_matrix, last_visit_matrix
        )
    elif algorithm == 'ucs':
        solution_node = ucs_search_algorithm(
            grid_map, start_position, end_position, number_of_rows, number_of_columns,
            visit_count_matrix, first_visit_matrix, last_visit_matrix
        )
    elif algorithm == 'astar':
        if heuristic_type not in ['euclidean', 'manhattan']:
            print("Error: A* requires 'euclidean' or 'manhattan' heuristic.")
            sys.exit(1)
        solution_node = astar_search_algorithm(
            grid_map, start_position, end_position, number_of_rows, number_of_columns,
            visit_count_matrix, first_visit_matrix, last_visit_matrix, heuristic_type
        )
    else:
        print(f"Error: Unknown algorithm '{algorithm}'. Use 'bfs', 'ucs', or 'astar'.")
        sys.exit(1)

    # Construct the path matrix for output
    if solution_node:
        path = reconstruct_path_from_goal_node(solution_node)
        path_set = set(path)
        path_matrix = [
            [
                ('*' if (row, col) in path_set else MAP_OBSTACLE_CHARACTER if grid_map[row][col] == MAP_OBSTACLE_VALUE else str(grid_map[row][col]))
                for col in range(number_of_columns)
            ]
            for row in range(number_of_rows)
        ]
    else:
        path_matrix = [['null']]

    # Output results based on mode
    if mode == 'debug':
        print("path:")  # Display the path or 'null' if no solution
        print_matrix_with_space_separation(
            path_matrix,
            number_of_rows if solution_node else 1,
            number_of_columns if solution_node else 1
        )

        print("#visits:")  # Number of times each cell was visited
        visits_matrix = [
            [
                'X' if grid_map[row][col] == MAP_OBSTACLE_VALUE else '.' if visit_count_matrix[row][col] == 0 else str(visit_count_matrix[row][col])
                for col in range(number_of_columns)
            ]
            for row in range(number_of_rows)
        ]
        print_matrix_with_space_separation(visits_matrix, number_of_rows, number_of_columns)

        print("first visit:")  # Step when each cell was first visited
        first_matrix = [
            [
                'X' if grid_map[row][col] == MAP_OBSTACLE_VALUE else '.' if first_visit_matrix[row][col] is None else str(first_visit_matrix[row][col])
                for col in range(number_of_columns)
            ]
            for row in range(number_of_rows)
        ]
        print_matrix_with_space_separation(first_matrix, number_of_rows, number_of_columns)

        print("last visit:")  # Step when each cell was last visited
        last_matrix = [
            [
                'X' if grid_map[row][col] == MAP_OBSTACLE_VALUE else '.' if last_visit_matrix[row][col] is None else str(last_visit_matrix[row][col])
                for col in range(number_of_rows)
            ]
            for row in range(number_of_rows)
        ]
        print_matrix_with_space_separation(last_matrix, number_of_rows, number_of_columns)
    elif mode == 'release':
        print("path:")  # Display only the path or 'null'
        print_matrix_with_space_separation(
            path_matrix,
            number_of_rows if solution_node else 1,
            number_of_columns if solution_node else 1
        )
    else:
        print(f"Error: Invalid mode '{mode}'. Use 'debug' or 'release'.")
        sys.exit(1)

if __name__ == "__main__":
    main()