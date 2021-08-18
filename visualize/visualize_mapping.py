#!/usr/bin/env python
import sys
import os
import os.path
import shutil
import xml.dom.minidom
import xml.etree.ElementTree as ET
import re
#from antlr4 import tree

import networkx as nx

from networkx.drawing.nx_pydot import write_dot
from networkx.drawing.nx_pydot import to_pydot
import pydot
from _ast import If

import numpy as np


from mpl_toolkits import mplot3d#https://towardsdatascience.com/an-easy-introduction-to-3d-plotting-with-matplotlib-801561999725
#matplotlib inline
import numpy as np
import matplotlib
matplotlib.use('TkAgg') #https://stackoverflow.com/questions/56656777/userwarning-matplotlib-is-currently-using-agg-which-is-a-non-gui-backend-so
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from numpy.random import rand
from pylab import figure


from mpl_toolkits.mplot3d.art3d import Line3DCollection
############################################
# Directory Structure:
# Morpher Home:
#     -Morpher_DFG_Generator
#     -Morpher_CGRA_Mapper
#     -hycube_simulator
#     -Morpher_Scripts

# Build all three tools before running this script
from mpl_toolkits.mplot3d.proj3d import proj_transform
from matplotlib.text import Annotation

class Annotation3D(Annotation):
    '''Annotate the point xyz with text s'''

    def __init__(self, s, xyz, *args, **kwargs):
        Annotation.__init__(self,s, xy=(0,0), *args, **kwargs)
        self._verts3d = xyz        

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.xy=(xs,ys)
        Annotation.draw(self, renderer)
        
def annotate3D(ax, s, *args, **kwargs):
    '''add anotation text s to to Axes3d ax'''

    tag = Annotation3D(s, *args, **kwargs)
    ax.add_artist(tag)
    
def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def my_mkdir(dir):
    try:
        os.makedirs(dir) 
    except:
        pass


def draw_3d():
    #xml_name = '/home/dmd/Workplace/HiMap2/Morpher_DFG_Generator/applications/aes/hycube_compilation/encrypt_INNERMOST_LN1_PartPred_DFG_without_clustering_1.xml'
    #xml_name = '//home/dmd/Workplace/Morphor/github_ecolab_repos/Morpher_DFG_Generator/applications/madgwick_fp_v2/MadgwickAHRSupdateIMU_INNERMOST_LN1_PartPred_DFG_1.xml'
    #xml_name = '/home/dmd/Workplace/HiMap2/Morpher_DFG_Generator/applications/edn/jpegdct_POST_LN111_PartPred_DFG_1.xml'
    #xml_name = '/home/dmd/Workplace/HiMap2/Morpher_DFG_Generator/applications/nettle-sha256/_nettle_sha256_compress_INNERMOST_LN1_PartPred_DFG_2.xml'
    #xml_name = '/home/dmd/Workplace/HiMap2/Morpher_DFG_Generator/applications/nsichneu/benchmark_body_INNERMOST_LN1_PartPred_DFG_1.xml'
    #xml_name = '/home/dmd/Workplace/HiMap2/Morpher_DFG_Generator/applications/picojpeg/idctRows_INNERMOST_LN1_PartPred_DFG_1.xml'
    #xml_name = '/home/dmd/Workplace/HiMap2/Morpher_DFG_Generator/applications/picojpeg/idctCols_INNERMOST_LN1_PartPred_DFG_1.xml'
    mapping_file = "../applications/hycube/array_add/array_add_INNERMOST_LN1_PartPred_DFG.xmlhycube_original.json_MTP\=1_Iter\=0.mappingwithlatency.txt"
    
    # clustering_outcome = '/home/dmd/Workplace/HiMap2/HiMap2_Scikit_Clustering/applications/edn/clustering_outcome.txt'
    #all_edges = '/home/dmd/Workplace/HiMap2/HiMap2_Scikit_Clustering/applications/edn/all_edges.txt'
    # all_edges = '/home/dmd/Workplace/HiMap2/HiMap2_Scikit_Clustering/applications/edn/inter_cluster_edges.txt'
    
    colordict = {
        0:'red',
        1:'green',
        2:'blue',
        3:'yellow',
        4:'cyan',
        5:'purple',
        6:'orange',
        7:'brown',
        8:'magenta',
        9:'rose',
        10:'azure'
        
    }
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')# Data for a three-dimensional line
    # zline = np.linspace(0, 15, 10)
    # xline = np.linspace(0, 10, 10) #np.sin(zline)
    # yline = np.linspace(0, 20, 10)#np.cos(zline)
    # ax.plot3D(xline, yline, zline, 'gray')
    #
    # z_points = 15 * np.random.random(100)
    # x_points = np.cos(z_points) + 0.1 * np.random.randn(100)
    # y_points = np.sin(z_points) + 0.1 * np.random.randn(100)
    # ax.scatter3D(x_points, y_points, z_points, c=z_points, cmap='hsv');
    # #plt.show()
    #
    # #plt.savefig("mygraph.png")
    # #https://stackoverflow.com/questions/10374930/matplotlib-annotating-a-3d-scatter-plot
    # m=rand(3,3) # m is an array of (x,y,z) coordinate triplets
    #
    # fig = figure()
    # ax = Axes3D(fig)
    #
    #
    # for i in range(len(m)): #plot each point + it's index as text above
    #     ax.scatter(m[i,0],m[i,1],m[i,2],color='b') 
    #     ax.text(m[i,0],m[i,1],m[i,2],  '%s' % (str(i)), size=20, zorder=1,  color='k') 
    #
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    #plt.show()
    
    # data: coordinates of nodes and links
    # xn = [1.1, 1.9, 0.1, 0.3, 1.6, 0.8, 2.3, 1.2, 1.7, 1.0, -0.7, 0.1, 0.1, -0.9, 0.1, -0.1, 2.1, 2.7, 2.6, 2.0]
    # yn = [-1.2, -2.0, -1.2, -0.7, -0.4, -2.2, -1.0, -1.3, -1.5, -2.1, -0.7, -0.3, 0.7, -0.0, -0.3, 0.7, 0.7, 0.3, 0.8, 1.2]
    # zn = [-1.6, -1.5, -1.3, -2.0, -2.4, -2.1, -1.8, -2.8, -0.5, -0.8, -0.4, -1.1, -1.8, -1.5, 0.1, -0.6, 0.2, -0.1, -0.8, -0.4]
    # group = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 2, 2, 2, 3, 3, 3, 3]
    edges = []#[(1, 0), (2, 0), (3, 0), (3, 2), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (11, 10), (11, 3), (11, 2), (11, 0), (12, 11), (13, 11), (14, 11), (15, 11), (17, 16), (18, 16), (18, 17), (19, 16), (19, 17), (19, 18)]
    # xyzn = list(zip(xn, yn, zn))
    # segments = [(xyzn[s], xyzn[t]) for s, t in edges]                
    #
    # # create figure        
    # fig = plt.figure(dpi=60)
    # ax = fig.gca(projection='3d')
    # #ax.set_axis_off()
    #
    # # plot vertices
    # ax.scatter(xn,yn,zn, marker='o', c = group, s = 64)    
    # # plot edges
    # edge_col = Line3DCollection(segments, lw=0.2)
    # ax.add_collection3d(edge_col)
    # # add vertices annotation.
    # for j, xyz_ in enumerate(xyzn): 
    #     annotate3D(ax, s=str(j), xyz=xyz_, fontsize=10, xytext=(-3,3),
    #            textcoords='offset points', ha='right',va='bottom')    
    #plt.show()
    # clustering_outcome

    lat = []
    node_id = []
    pe_x = []
    pe_y = []
    group = []
    node_id_to_index_Dict = {}
    
    mynumbers = []
    i=0
    with open(mapping_file) as f:
        for line in f:
            mynumbers.append([int(n) for n in line.strip().split(',')])
    for pair in mynumbers:
        try:
            #x,y = pair[0],pair[1]
            node_id_,x_,y_,lat_ = pair[0],pair[1],pair[2],pair[3]
            lat.append(lat_)
            node_id.append(node_id_)
            node_id_to_index_Dict[node_id_] = i
            pe_x.append(x_)
            pe_y.append(y_)
            group.append("")
            i = i+1
            print(node_id_,x_,y_,lat_ )
        # Do Something with x and y
        except IndexError:
            print("A line in the file doesn't have enough entries.")
            
    # mynumbers = []
    # with open(clustering_outcome) as f:
    #     for line in f:
    #         mynumbers.append([int(n) for n in line.strip().split('\t')])
    # for pair in mynumbers:
    #     try:
    #         #x,y = pair[0],pair[1]
    #         node_id_,dfg_cluster_id = pair[0],pair[1]
    #         group[node_id_to_index_Dict[node_id_]] = colordict[dfg_cluster_id]
    #         print(node_id_,dfg_cluster_id )
    #     # Do Something with x and y
    #     except IndexError:
    #         print("A line in the file doesn't have enough entries.")
    
    # mynumbers = []
    # with open(all_edges) as f:
    #     for line in f:
    #         mynumbers.append([int(n) for n in line.strip().split('\t')])
    # for pair in mynumbers:
    #     try:
    #         #x,y = pair[0],pair[1]
    #         edge_id_0,edge_id_1 = pair[0],pair[1]
    #         edges.append([node_id_to_index_Dict[edge_id_0],node_id_to_index_Dict[edge_id_1]])
    #         #print(node_id_,dfg_cluster_id )
    #     # Do Something with x and y
    #     except IndexError:
    #         print("A line in the file doesn't have enough entries.")        
    #with open(mapping_file) as f:
        #content = f.readlines()
        #print(content)
        #values = content.split(",")
        #int_values =  [int(e) for e in b]
        #node_id_,x_,y_,lat_ = int_values
        #lat.append(lat_)
        #node_id.append(node_id_)
        #pe_x.append(x_)
        #pe_y.append(y_)

    xyz = list(zip(pe_x, pe_y, lat))
    #segments = [(xyz[s], xyz[t]) for s, t in edges]  
    segments = [(xyz[s], xyz[t]) for s, t in edges]              

    # create figure        
    fig = plt.figure(dpi=60)
    ax = fig.gca(projection='3d')
    #ax.set_aspect('equal')
    #ax.set_axis_off()

    # plot vertices
    ax.scatter(pe_x,pe_y,lat, marker='o', s = 64)    # c = group, s = 64)    
    # plot edges
    edge_col = Line3DCollection(segments, lw=0.2)
    ax.add_collection3d(edge_col)
    # add vertices annotation.
    for j, xyz_ in enumerate(xyz): 
        annotate3D(ax, s=str(node_id[j]), xyz=xyz_, fontsize=10, xytext=(-3,3),
               textcoords='offset points', ha='right',va='bottom')    
    #set_axes_equal(ax) #https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
    ax.invert_zaxis()
    plt.show()
    plt.savefig("temp.png")
    
  


def draw_hot_pe(mapping_file):
    max_pe_x = 8
    max_pe_y = 8
    lat = []
    node_id = []
    pe_x = []
    pe_y = []
    mynumbers = []
    pe_utilization = [ [0]*max_pe_x for i in range(max_pe_y)]

    i=0
    with open(mapping_file) as f:
        for line in f:
            # print(line)
            mynumbers.append([int(n) for n in line.strip().split(',')])
    for pair in mynumbers:
        try:
            #x,y = pair[0],pair[1]
            node_id_,x_,y_,lat_ = pair[0],pair[1],pair[2],pair[3]
            lat.append(lat_)
            if node_id_ in node_id:
                continue
            node_id.append(node_id_)
            # node_id_to_index_Dict[node_id_] = i
            pe_x.append(x_)
            pe_y.append(y_)
            pe_utilization[x_][y_] +=1
            # group.append("")
            i = i+1
            print(node_id_,x_,y_,lat_ )
        # Do Something with x and y
        except IndexError:
            print("A line in the file doesn't have enough entries.")
        
    

    print(pe_utilization)

    fig, ax = plt.subplots()



    ax.matshow(pe_utilization, cmap=plt.cm.Blues)

    for i in range(max_pe_x):
        for j in range(max_pe_y):
            c = pe_utilization[i][j]
            ax.text(j, i, str(c), va='center', ha='center')
    

    plt.show()
    plt.savefig("temp.png")


if __name__ == "__main__":
    draw_hot_pe(sys.argv[1])

    