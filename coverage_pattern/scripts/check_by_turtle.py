#! /usr/bin/env python

import turtle as tt
import numpy as np
from shapely.geometry import Polygon

from libs.boustro_grid import create_grid as boustro_grid
from libs.boustro import create_path as boustro
from libs.zigzag_grid import create_grid as zigzag


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

def calculate_heading_angle(last_point, next_point):
    dx = next_point[0] - last_point[0]
    dy = next_point[1] - last_point[1]
    angle = np.degrees(np.arctan2(dy, dx))
    return angle

def test1(points, width=0.5, length=0.5):
    path = boustro_grid(points, width, length)
    #print(path)
    draw_path_with_turtle(points, path)


def test2(points, width=1):
    poly = Polygon(points)
    path = boustro(poly, width)
    #print(path)
    draw_path_with_turtle(points, path)
        
#test1([(0, 0), (0, 10), (10, 10), (10, 0)])
test2([(0, 0), (0, 10), (10,10), (10,0)])
#test2([[3.9339351654052734,2.0182785987854004],[4.099702835083008,-6.596142768859863],[0.9253101348876953,-6.6480488777160645],[0.8424863815307617,-1.9217795133590698],[1.1305875778198242,-1.038202166557312],[-0.4202451705932617,-0.880066990852356],[-0.7274589538574219,-1.397753119468689],[-0.536473274230957,-2.3324413299560547],[-3.526003837585449,-1.5986241102218628],[-3.7407665252685547,6.511865139007568],[-0.514704704284668,6.589041709899902],[-0.5950784683227539,2.7872300148010254],[-0.9175004959106445,1.5611735582351685],[-0.34500980377197266,0.7898861169815063],[0.7999715805053711,0.8164635896682739],[1.1601743698120117,1.6963587999343872],[3.844005584716797,1.9601703882217407]])