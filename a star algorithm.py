import heapq
import time
import matplotlib.pyplot as plt
import numpy as np
 
# different test cases 
TEST_GRIDS= {
    "Test Case 1: Simple Path": [
        "S . . . . . . .",
        ". . # # # . . .",
        ". . . . # . . .",
        ". # . . # . . .",
        ". # . . . . . .",
        ". . . . . . . G"
    ],

    
    "Test Case 2: Multiple Paths": [
        "S . . . . . . .",
        ". . # # # . . .",
        ". . # . # . . .",
        ". . # . # . . .",
        ". . # # # . . .",
        ". . . . . . . G"
    ],
    "Test Case 3: Narrow Passage": [
        "S . . # . . . .",
        ". . . # . . . .",
        "# # . # . # # #",
        ". . . . . . . .",
        ". . . # # # # .",
        ". . . . . . . G"
    ],
    "Test Case 4: Different Obstacles": [
        "S # . . . . . .",
        ". # . # # # # .",
        ". # . # . . . .",
        ". . . # . # # .",
        "# # # # . # G .",
        ". . . . . # . ."
    ],



    
    "Test Case 5: No Valid Path": [
        "S . . . . . . .",
        ". . # # # . . .",
        ". . # G # . . .",
        "# . # # # . . .",
        "# . . . . . . .",
        "# # . . . . # ."
    ]
}
 
DIRS= [(-1, 0), (1, 0), (0, -1), (0, 1)] #this changes the coordinates according to movement

 



 
def parse_grid(raw_rows): # raw rows is list of strings
    grid= []
    for row in raw_rows:
        
        grid.append(row.split())


        
 
    rows= len(grid)
    cols= len(grid[0]) # lenght of first list 
    start = None
    goal =None
 
    for i in range(rows):
        for j in range(cols):
            
            if grid[i][j]== 'S':
                start = (i, j)

            elif grid[i][j]== 'G':
                goal = (i, j)
 
    return grid, start, goal, rows, cols



 
# grid is list of list with each character seperated along white space 
def heuristic(a, b): # a and b are tuples
    # here Manhatten distance is better to use as no diagonal movements are allowed
    
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
 
 
def astar(grid, start, goal, rows, cols):
    t_start = time.time()
 
    pq =[(0, start)] # pq is used for priority que 
    cost= {start: 0} 
    parent= {} # this will contain the current and previous cell 
    seen = set()
    found =False # found will be turned TRUE when g is found

    
 
    while pq :
        f, node = heapq.heappop(pq)# f stores priority no. and node its coordinates
        
 
        if node in seen:
            continue
        seen.add(node)
 
        if node== goal:
            found= True
            break
 



        for dr, dc in DIRS: # dr and dc are row and column change for moving in perticular direction
        
            nr,nc =node[0] +dr, node[1] + dc # nr adn nc are neighnbour rows and columsns
    
 
            if nr <0 or nr >= rows or nc <0 or nc >= cols: # check weather new cell is in grid or not
                continue # check 4 near cells
            if grid[nr][nc]== '#':
                continue
 
            new_cost= cost[node] + 1 # fore new near cell
            neighbor= (nr, nc)
 





            if neighbor not in cost or new_cost < cost[neighbor]:


                cost[neighbor]= new_cost
                parent[neighbor]=node
                heapq.heappush(pq, (new_cost + heuristic(neighbor, goal), neighbor))
 
    time_taken =time.time() - t_start








    
 
    # this is use dto  trace the path , and get the coordinates
    path = []

    
    if found:
        node = goal
        
        while node !=start  :
            path.append(node)
            node = parent[node]# uses saved dictionary to check previous node
        path.append(start)
        path.reverse()
 
    return found,path,len(seen),time_taken, seen
 
 
def print_result(name, found, path, explored, time_taken, filename):
    print("\n---Search Result---")
    






    if found:
        print(f"{name} Path Found: YES")
        print("Path:")
        for r, c in path:
            print(f"({r},{c})")
        print(f"Total Path Cost: {len(path) - 1} Nodes Explored: {explored}")



    else:
        print(f"{name} Path Found: NO")
        print("No valid path exists between Start and Goal.")
        print(f"Nodes Explored: {explored}")
    print(f"Execution Time: {time_taken:.6f} seconds Visualization saved: {filename}")
 
















 
def draw_grid(grid, path, seen, start, goal, name, filename):
    rows =len(grid)
    cols =len(grid[0])
 
    # displayig grid 
    obs =np.zeros((rows, cols))
    for i in range(rows):
        for j in range(cols):
            if grid[i][j]=='#':
                obs[i][j]  =1



 
    fig,ax =plt.subplots(figsize=(6, 6))
    ax.imshow(obs, cmap='binary',origin='upper')
 
    if seen:
        xs =[c for r, c in seen]
        ys =[r for r, c in seen]
        ax.scatter(xs, ys, color='lightblue', marker='s', s=150, alpha=0.6, label='Explored')

 
    if path:
    

        
        xs =[c for r, c in path]
        ys = [r for r, c in path]
        ax.plot(xs, ys, color='orange', linewidth=3, label='Path')
        ax.scatter(xs, ys, color='orange', s=50)
 
    ax.scatter([start[1]],  [start[0]], color='green', s=150, zorder=5, label='Start (S)')
    ax.scatter([goal[1]], [goal[0]], color='red', s=150, zorder=5, label='Goal (G)')




 
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.grid(color='gray', linestyle=':', linewidth=0.5)
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))
    ax.set_title(name)

    
 
    plt.tight_layout()
    plt.savefig(filename, bbox_inches="tight")
    plt.close()
 



 
if __name__ =="__main__":
    count = 1

    
    for name, raw_rows in TEST_GRIDS.items():
        filename =f"testcase_{count}.png"
 


        grid, start, goal, rows, cols = parse_grid(raw_rows)
        found, path, explored, time_taken, seen = astar(grid, start, goal, rows, cols)
 

        print_result(name, found, path, explored, time_taken, filename)
        draw_grid(grid, path, seen, start, goal, name, filename)
 
        count+=1
