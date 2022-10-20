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

    v = []
    for d in data:
        key, val = d
        if val[0][0] == 'twist':
            # Twist: X : val[0][1][0][1][0][1] | Y: val[0][1][0][1][1][1]
            v.append( math.sqrt( pow(val[0][1][0][1][0][1], 2) + pow(val[0][1][0][1][1][1], 2) ))

    return v


def plot_line(v, name, color, x_label="", y_label=""):

    # Latex inforamtion for pgf
    mpl.rcParams.update({
        'pgf.texsystem': 'lualatex',
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
    })
    mpl.rc('font', size=12)

    # Plot size
    fig, ax = plt.subplots(figsize=(10, 3))

    X = np.arange(0,len(v))
    ax.plot(X, v, "b", linewidth=1)

    x_ax = np.zeros(len(v))

    ax.set_yscale('linear')
    ax.set_xscale('linear')

    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)

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

    v = load_data()

    plot_line(v, "velocity_profile", "r")
