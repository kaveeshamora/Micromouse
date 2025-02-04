import API
import sys
import time

def update_direction_glob_at_rotate_right(direction_glob):
    first_dir = direction_glob[0]
    for i in range(0, 3, 1):
       direction_glob[i] = direction_glob[i+1] 
    direction_glob[-1] = first_dir
    return direction_glob

def update_direction_glob_at_rotate_left(direction_glob):
    last_dir = direction_glob[-1]
    for i in range(3, 0, -1):
       direction_glob[i] = direction_glob[i-1] 
    direction_glob[0] = last_dir
    return direction_glob

def check_availability():
    leftwall_available = API.wallLeft()
    frontwall_available = API.wallFront()
    rightwall_available = API.wallRight()

    return leftwall_available, frontwall_available, rightwall_available

def update_map(leftwall_available, frontwall_available, rightwall_available, current_row, current_col):
    if leftwall_available:
        glob_dir = direction_glob[0]
        API.setWall(current_col, current_row, glob_dir)
    if frontwall_available:
        glob_dir = direction_glob[1]
        API.setWall(current_col, current_row, glob_dir)
    if rightwall_available:
        glob_dir = direction_glob[2] 
        API.setWall(current_col, current_row, glob_dir)      

def update_mouse_pos(current_row, current_col):
    row = current_row
    col = current_col

    global_front_dir = direction_glob[1]
    if global_front_dir == 'n':
        current_row = row + 1
    elif global_front_dir == 'e':
        current_col = col + 1 
    elif global_front_dir == 's':
        current_row = row - 1 
    elif global_front_dir == 'w':
        current_col = col - 1 

    return current_row, current_col

def update_matrix(current_col, current_row, wall_matrix, leftwall_available, frontwall_available, rightwall_available, direction_glob):
    
    wall_array = [0, 0, 0, 0]
    if leftwall_available:
        glob_dir = direction_glob[0]
        if glob_dir == 'w':
           wall_array[0] = 1 
        elif glob_dir == 'n':
           wall_array[1] = 1 
        elif glob_dir == 'e':
           wall_array[2] = 1 
        elif glob_dir == 's':
           wall_array[3] = 1  
    if frontwall_available:
        glob_dir = direction_glob[1]
        if glob_dir == 'w':
           wall_array[0] = 1 
        elif glob_dir == 'n':
           wall_array[1] = 1 
        elif glob_dir == 'e':
           wall_array[2] = 1 
        elif glob_dir == 's':
           wall_array[3] = 1  
    if rightwall_available:
        glob_dir = direction_glob[2]
        if glob_dir == 'w':
           wall_array[0] = 1 
        elif glob_dir == 'n':
           wall_array[1] = 1 
        elif glob_dir == 'e':
           wall_array[2] = 1 
        elif glob_dir == 's':
           wall_array[3] = 1

    wall_matrix[current_row][current_col] = wall_array

    return wall_matrix    

def check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, check_direction):

    next_row = current_row
    next_col = current_col

    if check_direction == "left":
        glob_dir = direction_glob[0]
    elif check_direction == "forward":
        glob_dir = direction_glob[1]
    elif check_direction == "right":
        glob_dir = direction_glob[2]    
    
    if glob_dir == 'n':
        next_row = next_row + 1
    elif glob_dir == 'e':
        next_col = next_col + 1 
    elif glob_dir == 's':
        next_row = next_row - 1 
    elif glob_dir == 'w':
        next_col = next_col - 1 

    if wall_matrix[next_row][next_col][0] == -1:
        return True
    else:
        return False  

def goto_stack_prev_pos(prev_pos, current_row, current_col, direction_glob):
    log("went into define funcction")
    log(f"prev pos {prev_pos}")
    log(f"current pos {current_col}, {current_row}")
    if (prev_pos[1] == current_row - 1):
        log("go to down")
        if direction_glob[1] == 'e':
            API.turnRight()
            direction_glob = update_direction_glob_at_rotate_right(direction_glob)
        elif direction_glob[1] == 'w':
            API.turnLeft()
            direction_glob = update_direction_glob_at_rotate_left(direction_glob)
        elif direction_glob[1] == 's':
            pass

    elif (prev_pos[1] == current_row + 1):
        log("go to up")
        if direction_glob[1] == 'e':
            API.turnLeft()
            direction_glob = update_direction_glob_at_rotate_left(direction_glob)
        elif direction_glob[1] == 'w':
            API.turnRight()
            direction_glob = update_direction_glob_at_rotate_right(direction_glob)
        elif direction_glob[1] == 'n':
            pass
    
    elif (prev_pos[0] == current_col + 1):
        log("go to right")
        if direction_glob[1] == 'n':
            API.turnRight()
            direction_glob = update_direction_glob_at_rotate_right(direction_glob)
        elif direction_glob[1] == 's':
            API.turnLeft()
            direction_glob = update_direction_glob_at_rotate_left(direction_glob)
        elif direction_glob[1] == 'e':
            pass
    
    elif (prev_pos[0] == current_col - 1):
        log("go to left")
        if direction_glob[1] == 's':
            API.turnRight()
            direction_glob = update_direction_glob_at_rotate_right(direction_glob)
        elif direction_glob[1] == 'n':
            API.turnLeft()
            direction_glob = update_direction_glob_at_rotate_left(direction_glob)
        elif direction_glob[1] == 'w':
            pass

    return direction_glob
    

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():

    # left = 0, forward = 1, right = 2, back = 3
    # when rotate to right => shift to left
    # when rotate to left => shift to right 

    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "start")

    global direction_glob
    direction_glob = ['w', 'n', 'e', 's']

    current_row = 0
    current_col = 0

    direction_stack = []
    # wall_matrix = [ 
    #                 [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
    #                 [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
    #                 [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
    #                 [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]]
    #               ]
    
    wall_matrix = [
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]], 
                    [[-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, -1]]
                    ]

    end = False 
    API.setWall(0, 0, "s")

    # traversing
    while True:

        leftwall_available, frontwall_available, rightwall_available = check_availability()
        update_map(leftwall_available, frontwall_available, rightwall_available, current_row, current_col)

        # decision
        direction_stack.append([current_col, current_row])
        wall_matrix = update_matrix(current_col, current_row, wall_matrix, leftwall_available, frontwall_available, rightwall_available, direction_glob)

        # decide next command - Prioritiesed array => forward, right, left
        next_step_launched = False
        if ((not frontwall_available) and (not next_step_launched)):
            if check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, "forward"):
                next_step_launched = True

        if ((not rightwall_available) and (not next_step_launched)):
            if check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, "right"):
                next_step_launched = True
                API.turnRight()
                direction_glob = update_direction_glob_at_rotate_right(direction_glob)

        if ((not leftwall_available) and (not next_step_launched)):
            if check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, "left"):
                next_step_launched = True
                API.turnLeft()
                direction_glob = update_direction_glob_at_rotate_left(direction_glob)

        # if (frontwall_available and (not next_step_launched)):
        if not next_step_launched:
            API.turnRight()
            direction_glob = update_direction_glob_at_rotate_right(direction_glob)
            API.turnRight()
            direction_glob = update_direction_glob_at_rotate_right(direction_glob)

            while True:
                API.moveForward()
                current_row, current_col = update_mouse_pos(current_row, current_col)
                
                if (current_col == 0 and current_row == 0):
                    end = True
                    break

                direction_stack.pop()
                prev_pos = direction_stack[-2]

                leftwall_available, frontwall_available, rightwall_available = check_availability()

                if not frontwall_available:
                    if check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, "forward"):
                        log("no prev traverse")
                        break

                if not rightwall_available:
                    if check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, "right"):
                        log("no prev traverse")
                        API.turnRight()
                        direction_glob = update_direction_glob_at_rotate_right(direction_glob)
                        break

                if not leftwall_available:
                    if check_not_traversed_before(current_row, current_col, wall_matrix, direction_glob, "left"):
                        log("no prev traverse")
                        API.turnLeft()
                        direction_glob = update_direction_glob_at_rotate_left(direction_glob)
                        break

                log("previously traversed")
                log(f"{direction_stack}")
                direction_glob = goto_stack_prev_pos(prev_pos, current_row, current_col, direction_glob)

        if end == True:
            log("end of traversed")
            break
        else:
            API.moveForward()
            current_row, current_col = update_mouse_pos(current_row, current_col)

            # check for the goal
            # if current_col == 1 and current_row == 2:
            if current_col == 5 and current_row == 3:
                log("arrived to goal") 
                goal_pos = [current_col, current_row]

    wall_matrix[0][0][3] = 1
    log("\nSearching is completed.\n")
    for row in range(9):
        main_string = "{"
        for col in range(9):
            main_string += "{"
            for item in range(4):
                if item == 3:
                    main_string += f"{wall_matrix[row][col][item]}"
                else:
                    main_string += f"{wall_matrix[row][col][item]}, "

            if col == 9:
                main_string += "}"
            else:
                main_string += "}, "
        
        if row == 9:
            main_string += "}"
        else:
            main_string += "}, "
            
        log(f"{main_string}") 


            
                

    time.sleep(5)

    # solving the maze
    log("\nSolving started.\n")

    flood_fill_stack = []
    # flood_fill_matrix = [
    #                         [-1, -1, -1, -1], 
    #                         [-1, -1, -1, -1], 
    #                         [-1, -1, -1, -1], 
    #                         [-1, -1, -1, -1]
    #                     ]
    
    flood_fill_matrix = [
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1],
                            [-1, -1, -1, -1, -1, -1, -1, -1, -1]
                        ]
    
    log(f"{flood_fill_matrix}")
    
    # Planning floodfill
    current_pos = goal_pos
    flood_fill_stack.append([*current_pos, 0])
    flood_fill_matrix[current_pos[1]][current_pos[0]] = 0
    log(f"{flood_fill_stack}")

    while True:
        stack_element = flood_fill_stack[0]
        stack_element_val = stack_element[2]

        wall_config = wall_matrix[stack_element[1]][stack_element[0]]

        # if there is a wall in west or north or east or south
        if wall_config[0] == 0:
            changing_pos = [stack_element[0] - 1, stack_element[1], stack_element_val+1]
            if flood_fill_matrix[changing_pos[1]][changing_pos[0]] == -1:
                flood_fill_stack.append(changing_pos)
                flood_fill_matrix[changing_pos[1]][changing_pos[0]] = changing_pos[2]
        if wall_config[1] == 0:
            changing_pos = [stack_element[0], stack_element[1] + 1, stack_element_val+1]
            if flood_fill_matrix[changing_pos[1]][changing_pos[0]] == -1:
                flood_fill_stack.append(changing_pos)
                flood_fill_matrix[changing_pos[1]][changing_pos[0]] = changing_pos[2]
        if wall_config[2] == 0:
            changing_pos = [stack_element[0] + 1, stack_element[1], stack_element_val+1]
            if flood_fill_matrix[changing_pos[1]][changing_pos[0]] == -1:
                flood_fill_stack.append(changing_pos)
                flood_fill_matrix[changing_pos[1]][changing_pos[0]] = changing_pos[2]
        if wall_config[3] == 0:
            changing_pos = [stack_element[0], stack_element[1] - 1, stack_element_val+1]
            if flood_fill_matrix[changing_pos[1]][changing_pos[0]] == -1:
                flood_fill_stack.append(changing_pos)
                flood_fill_matrix[changing_pos[1]][changing_pos[0]] = changing_pos[2]

        log(f"{flood_fill_matrix}")

        flood_fill_stack = flood_fill_stack[1:]
        if (len(flood_fill_stack) == 0):
            break

    log("\nSolving ended.\n")
    log(f"{flood_fill_matrix}")

    # travelling through solved path
    log(f"{direction_glob}")
    
    API.turnRight()
    direction_glob = update_direction_glob_at_rotate_right(direction_glob)
    API.turnRight()
    direction_glob = update_direction_glob_at_rotate_right(direction_glob)

    current_pos = [0, 0]

    while True:
        current_val = flood_fill_matrix[current_pos[1]][current_pos[0]]

        # check for west side square
        if current_pos[0] - 1 >= 0 and flood_fill_matrix[current_pos[1]][current_pos[0] - 1] == current_val - 1 and wall_matrix[current_pos[1]][current_pos[0]][0] != 1:
            log("west side")
            if direction_glob[1] == 's':
                API.turnRight()
                direction_glob = update_direction_glob_at_rotate_right(direction_glob)
            elif direction_glob[1] == 'w':
                pass
            elif direction_glob[1] == 'n':
                API.turnLeft()
                direction_glob = update_direction_glob_at_rotate_left(direction_glob)
        # check for north side square        
        elif current_pos[1] + 1 <= 8 and flood_fill_matrix[current_pos[1] + 1][current_pos[0]] == current_val - 1 and wall_matrix[current_pos[1]][current_pos[0]][1] != 1:
            log("north side")
            if direction_glob[1] == 'w':
                API.turnRight()
                direction_glob = update_direction_glob_at_rotate_right(direction_glob)
            elif direction_glob[1] == 'n':
                pass
            elif direction_glob[1] == 'e':
                API.turnLeft()
                direction_glob = update_direction_glob_at_rotate_left(direction_glob) 
        # check for east side square        
        elif current_pos[0] + 1 <= 8 and flood_fill_matrix[current_pos[1]][current_pos[0] + 1] == current_val - 1 and wall_matrix[current_pos[1]][current_pos[0]][2] != 1:
            log("east side")
            if direction_glob[1] == 'n':
                API.turnRight()
                direction_glob = update_direction_glob_at_rotate_right(direction_glob)
            elif direction_glob[1] == 'e':
                pass
            elif direction_glob[1] == 's':
                API.turnLeft()
                direction_glob = update_direction_glob_at_rotate_left(direction_glob) 
        # check for south side square        
        elif current_pos[1] - 1 >= 0 and flood_fill_matrix[current_pos[1] - 1][current_pos[0]] == current_val - 1 and wall_matrix[current_pos[1]][current_pos[0]][3] != 1:
            log("south side")
            if direction_glob[1] == 'e':
                API.turnRight()
                direction_glob = update_direction_glob_at_rotate_right(direction_glob)
            elif direction_glob[1] == 's':
                pass
            elif direction_glob[1] == 'w':
                API.turnLeft()
                direction_glob = update_direction_glob_at_rotate_left(direction_glob) 

        row, col = update_mouse_pos(current_pos[1], current_pos[0])
        current_pos = [col, row]
        log(f"{current_pos}")
        API.moveForward()

        if current_pos[0] == goal_pos[0] and current_pos[1] == goal_pos[1]:
            break
        
    log("Finished arriving to the goal")

if __name__ == "__main__":
    main()