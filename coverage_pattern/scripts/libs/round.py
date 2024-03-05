#!/usr/bin/env python

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from math import *

def calc_path2(poly, robot_width):
	ret = []

	# Decrease width to integer number of runs
	poly_width = poly.bounds[2]-poly.bounds[0]
	poly_height = poly.bounds[3]-poly.bounds[1]

	if poly_width < robot_width and poly_height < robot_width:
		return ret # Don't bother
	
	if robot_width >= poly_width or robot_width >= poly_height:
		# if smaller then width try at least one sweep
		xscale = 1-(robot_width/poly_width) if robot_width < poly_width else 0.0
		yscale = 1-(robot_width/poly_height) if robot_width < poly_height else 0.0
		poly = shapely.affinity.scale(poly, xscale, yscale)

	plows = ceil(poly_width/(robot_width))
	plow_width = poly_width/plows
	print(plows, plow_width)

	if poly.is_empty:
		# No polygon left
		return ret
	waypoints = []
	for x in range(1, plows):
		# Reduce size of polygon by width
		inner_radius = x * robot_width
		buf_polygon = poly.buffer(-inner_radius)
	
        # Collect the exterior coordinates of buf_polygons
		xypoint = list(buf_polygon.exterior.coords)
		waypoints.extend(xypoint)
		path = convert_to_list(waypoints)
	
	return path

def convert_to_list(waypoints):
	return [list(point) for point in waypoints]

if __name__ == "__main__":
	import matplotlib.pyplot as plt
	import matplotlib.patches
	def paint_arrow(ax, pos_start, pos_end, width):
		length_x = pos_end[0]-pos_start[0]
		length_y = pos_end[1]-pos_start[1]
		ax.arrow(pos_start[0], pos_start[1], length_x, length_y, color='r', width=width/10, length_includes_head=True, shape='right')
		ax.arrow(pos_start[0], pos_start[1], length_x, length_y, color='r', width=width/10, length_includes_head=True, shape='left')
		
	def test(points, width=0.1):
		fig, ax = plt.subplots()
		poly = Polygon(points)
		path = calc_path2(poly, width)
		print(path)
		pos_last = None
		for pos_cur in path:
			if pos_last is not None:
				paint_arrow(ax, pos_last, pos_cur, width)
			pos_last = pos_cur
		ax.axis([poly.bounds[0]-width*2, poly.bounds[2]+width*2, poly.bounds[1]-width*2, poly.bounds[3]+width*2])
		ax.add_patch(matplotlib.patches.Polygon(points))
		plt.gca().set_aspect('equal', adjustable='box')
        
		
		x_coords, y_coords = zip(*path)
		plt.scatter(x_coords, y_coords, label='waypoints', color='red', marker='o')
		plt.grid()
		plt.show()
		
	test([[0.0, 0.0], [2.0, 0.0], [2.0, 1.5], [0.0, 1.5]])
	test([[0.0, 0.3], [1.0, 0.1], [1.0, 1.5], [0.0, 1.2]])