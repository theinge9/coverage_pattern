#! /usr/bin/env python
"""
Boustrophedon Pattern for polygon grid cells
"""
import numpy as np
import matplotlib.pyplot as plt
import turtle as tt
from shapely.geometry import Polygon, Point

def create_grid(points, robot_x, robot_y):
    ## Define the polygon ##
    poly = Polygon(points)
    poly_width = poly.bounds[2]-poly.bounds[0]
    poly_height = poly.bounds[3]-poly.bounds[1]

    print("min_x: ",poly.bounds[0],"min_y: ",poly.bounds[1])
    print("max_x: ",poly.bounds[2], "max_y: ",poly.bounds[3])
    
    grid_cells = []
    row_counts = 0

    if poly_width < robot_x and poly_height < robot_y:
        return grid_cells
    
    for i in np.arange(poly.bounds[0]+robot_x/2, poly.bounds[2], robot_x):
        row_cells = []
        for j in np.arange(poly.bounds[1]+robot_y/2, poly.bounds[3], robot_y):
            x = i
            y = j   
    
            if poly.contains(Point(x,y)):
                row_cells.append((x,y))
        
                if row_counts % 2 == 0:
                    row_cells.sort(key=lambda cell: cell[1])
                else:
                    row_cells.sort(key=lambda cell: cell[1], reverse=True)

        grid_cells.extend(row_cells)
        row_counts += 1

    #print(grid_cells)
    return grid_cells

def calculate_heading_angle(last_point, next_point):
    dx = next_point[0] - last_point[0]
    dy = next_point[1] - last_point[1]
    angle = np.degrees(np.arctan2(dy, dx))
    return angle

def plot_grid(points, grid_cells):
    ## Visualize the results ##
    fig, ax = plt.subplots()
    x, y = zip(*points)
    ax.plot(x + (x[0],), y + (y[0],), linestyle='-', color='blue', label='Polygon')

    if grid_cells:
        x, y = zip(*grid_cells)
        plt.scatter(x, y, c='red', label='Grid Cells', marker='s')

        ## Connect the grid cells to show path ##
        for i in range(len(grid_cells)-1):
            plt.plot([grid_cells[i][0], grid_cells[i+1][0]], [grid_cells[i][1], grid_cells[i+1][1]], c='green')
    
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Grid Cells within Polygon')
    plt.legend()
    plt.show()

def draw_path_with_turtle(points, grid_cells):
    screen = tt.Screen()
    tt.shape("triangle")

    ## Set visible coordinate range ##
    tt.setworldcoordinates(min(cell[0] for cell in grid_cells) -1,
                               min(cell[1] for cell in grid_cells) -1,
                               max(cell[0] for cell in grid_cells) +1,
                               max(cell[1] for cell in grid_cells) +1)
    #tt.setworldcoordinates(-10,-10,10,10)

    ## Boarder Drive ##
    last_point = points[0]
    for i,current_point in enumerate(points):
        tt.color('red')
        tt.setheading(calculate_heading_angle(last_point, current_point))
        tt.goto(current_point[0], current_point[1])
        tt.pendown()
        screen.title("Boarder_Drive")
        last_point = current_point
    screen.title("Boarder_Drive")
    tt.setheading(calculate_heading_angle(last_point, points[0]))
    tt.goto(points[0])

    ## Pattern Drive ##
    total_cells = len(grid_cells)
    last_cell = grid_cells[0]
    for i, current_cell in enumerate(grid_cells):
        tt.color('green')
        tt.setheading(calculate_heading_angle(last_cell, current_cell))
        tt.goto(current_cell[0] , current_cell[1])
        
        tt.pendown()
        #tt.dot(100)
    
        ## Calculate percentage ##
        percentage = (i + 1) / total_cells * 100
        screen.title(f"Travel Percentage: {percentage:.2f}%")

        last_cell = current_cell

        ## Check 100% or not ##
        if percentage == 100:
            screen.title(" Yayy! 100% ")
        
    tt.done()

if __name__ == "__main__":
    def test(points, width=0.1, length=0.1):
        grid_cells = create_grid(points, width, length)
        plot_grid(points, grid_cells)
        #draw_path_with_turtle(points, grid_cells)
    
    test([(0, 0), (0, 10), (10, 10), (10, 0)])
    test([(1.0248732566833496, -1.0239896774291992), (4.030491352081299, -1.0422382354736328), (4.034761428833008, -5.011183261871338), (1.0295500755310059, -5.014775276184082), (1.0467060804367065, -1.0457961559295654)])
    test([[3.9339351654052734,2.0182785987854004],[4.099702835083008,-6.596142768859863],[0.9253101348876953,-6.6480488777160645],[0.8424863815307617,-1.9217795133590698],[1.1305875778198242,-1.038202166557312],[-0.4202451705932617,-0.880066990852356],[-0.7274589538574219,-1.397753119468689],[-0.536473274230957,-2.3324413299560547],[-3.526003837585449,-1.5986241102218628],[-3.7407665252685547,6.511865139007568],[-0.514704704284668,6.589041709899902],[-0.5950784683227539,2.7872300148010254],[-0.9175004959106445,1.5611735582351685],[-0.34500980377197266,0.7898861169815063],[0.7999715805053711,0.8164635896682739],[1.1601743698120117,1.6963587999343872],[3.844005584716797,1.9601703882217407]])
    test([[0.0, 0.3], [1.0, 0.3], [1.0, -1.2], [0.0, -1.2]])
    test([[0.5, 0.5], [2.0, 0.3], [1.5, 1.5], [0.0, 1.5]])