#!/usr/bin/env python3

# For Argument inputs
import sys, getopt
thismodule = sys.modules[__name__]

# Handle ROSBAG data from yaml
import yaml
from ruamel.yaml import YAML
from ruamel.yaml.constructor import SafeConstructor

import math
import numpy as np

# Create Plot
from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.collections import LineCollection
mpl.use("pgf")

########################################################
#############           CONFIG          ################
########################################################
# If you dont set by arguments
thismodule.file_name = "" 

path = "./yaml/"
final_path = "./plots/"
########################################################
########################################################
########################################################

def construct_yaml_map(self, node):
    # test if there are duplicate node keys
    data = []
    yield data
    for key_node, value_node in node.value:
        key = self.construct_object(key_node, deep=True)
        val = self.construct_object(value_node, deep=True)
        data.append((key, val))


def load_data():
    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')
    try:
        with open(path + thismodule.file_name  , 'r') as stream:
            data = yaml.load(stream)
    except FileNotFoundError:
        print("ERROR: File Not Found")
        sys.exit(2)
    except IsADirectoryError:
        print("ERROR: Wrong Path or filename")
        sys.exit(2)
    except:
        print("ERROR: Unknown")
        sys.exit(2)

    x = []
    y = []
    v = []
    for d in data:
        key, val = d
        if val[0][0] == 'twist':
            # Twist: X : val[0][1][0][1][0][1] | Y: val[0][1][0][1][1][1]
            v.append( math.sqrt( pow(val[0][1][0][1][0][1], 2) + pow(val[0][1][0][1][1][1], 2) ))

        if val[0][0] == 'pose':
            x.append(val[0][1][0][1][0][1])
            y.append(val[0][1][0][1][1][1])

    dist = 0
    for i in range(1, len(x)):
        dist = dist + math.sqrt( pow(x[i] - x[i-1],2) + pow( y[i] - y[i-1],2))

    return x, y, v, round(dist, 2)

def plot_line(x, y, vel, dist, name,  x_label="", y_label=""):

    # Latex inforamtion for pgf
    mpl.rcParams.update({
        'pgf.texsystem': 'lualatex',
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
    })
    mpl.rc('font', size=12)

    cols = np.array(vel)
    
    points = np.array([np.array(x), np.array(y)]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    fig, ax = plt.subplots()
    lc = LineCollection(segments, cmap='plasma')
    lc.set_array(cols)
    lc.set_linewidth(2)
    line = ax.add_collection(lc)
    fig.colorbar(line, ax=ax)

    plt.xticks(range(-3, 4))
    plt.yticks(range(-3, 4))

    plt.xlabel(x_label) 
    plt.ylabel(y_label) 
    
    # displaying the title
    plt.title("Distance traveled: ~" + str(dist) + "m   -   A*")

    plt.savefig(final_path + name + '_' + thismodule.file_name + '.pgf', bbox_inches = 'tight')
    plt.savefig(final_path + name + '_' + thismodule.file_name +'.png',dpi=500, bbox_inches = 'tight')


if __name__ == '__main__':

    try:
        if not len(sys.argv) == 1:
            opt, _ = getopt.getopt(sys.argv[1:],"hf:",["file="])
        else : 
            opt = None
    except getopt.GetoptError:
        print('plot_cmd.py -f <file>')
        sys.exit(2)

    if opt != None:
        if opt[0][0] in ("-f", "--file"):
            thismodule.file_name = opt[0][1]
        else: 
            print("WRONG PARAMTER: USE -f or --file")
            sys.exit(2)

    x, y, vel, dist = load_data()

    plot_line(x, y, vel, dist, "odom_with_speed", "x coordinate", "y coordinate")
    # plot_line(rot, "rotation", "g")
