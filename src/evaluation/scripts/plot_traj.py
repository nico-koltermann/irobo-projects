import yaml


import sys
from ruamel.yaml import YAML
from ruamel.yaml.constructor import SafeConstructor

from scipy import interpolate

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib as mpl
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
mpl.use("pgf")               # to use xelatex

import math

def construct_yaml_map(self, node):
    # test if there are duplicate node keys
    data = []
    yield data
    for key_node, value_node in node.value:
        key = self.construct_object(key_node, deep=True)
        val = self.construct_object(value_node, deep=True)
        data.append((key, val))


def load_map():
    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')
    with open("map.yaml", 'r') as stream:
        data = yaml.load(stream)

    x_pos = []
    y_pos = []
    for x in data[1][1]:

        for y in x:
            if y[0] == 'position':
                x_pos.append(y[1][0][1])
                y_pos.append(y[1][1][1])

    
        
    return x_pos, y_pos

def load_car():

    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')
    with open("new.yaml", 'r') as stream:
        data = yaml.load(stream)

    x_pos = []
    y_pos = []
    data = data
    for x in data:
        for y in x:
            try:
                if str(y[0][0]) == 'x':
                    x_pos.append(y[0][1])
                    y_pos.append(y[1][1])
            except:
                pass

    return x_pos, y_pos

def load_car_dt():

    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')
    with open("new.yaml", 'r') as stream:
        data = yaml.load(stream)

    [('header', [('seq', 1661), ('stamp', [('secs', 1350), ('nsecs', 37000000)]), ('frame_id', 'fssim_map')]), ('car_state_dt', [('x', -0.00550927497735), ('y', -0.022434952039), ('theta', 0.00627926128868)])]

    x_pos = []
    y_pos = []
    data = data
    for x in data:
        for y in x:
            try:
                if str(y[1][1][0][0]) == 'x':
                    x_pos.append(y[1][1][0][1])
                    y_pos.append(y[1][1][1][1])
            except:
                pass

    return x_pos, y_pos

def load_trj():

    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')
    with open("trj.yaml", 'r') as stream:
        data = yaml.load(stream)

    x_pos = []
    y_pos = []
    data = data
    for x in data:
        for y in x:
            for z in y:
                try:
                    if str(z[0][0]) == 'x':
                        x_pos.append(z[0][1])
                        y_pos.append(z[1][1])
                except:
                    pass

    return x_pos, y_pos
    

# [('id', [('data', 177)]), ('position', [('x', -15.5632275236), ('y', -8.86714013321), ('z', 0.0)]), ('color', [('data', 98)])]

# [
# ('header', [('seq', 701), ('stamp', [('secs', 1354), ('nsecs', 287000000)]), ('frame_id', 'fssim_map')]), 
# ('car_state', [('x', 0.0), ('y', 0.0), ('theta', 0.0)]), 
# ('car_state_dt', [('header', [('seq', 2583), ('stamp', [('secs', 1354), ('nsecs', 632000000)]), ('frame_id', 'fssim_map')]), ('car_state_dt', [('x', 0.00525326881448), ('y', 0.00459559201341), ('theta', 0.0130316308921)])])

def plot(x_pos, y_pos, x_car, y_car, x_trj, y_trj, vel):

    mpl.rcParams.update({
    "pgf.texsystem": "lualatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
    # "pgf.preamble": [
    #      "\\usepackage{unicode-math}",  
    #      r"\setmathfont{libertinusmath-regular.otf}",
    #      r"\setmainfont{libertinusserif-regular.otf}",
    #      r"\setsansfont{libertinussans-regular.otf}",
    #      ]
    })

    mpl.rc('font', size=12)


    fig, ax = plt.subplots()



    ax.plot(x_pos[:int(len(x_pos)/2 + 3)], y_pos[:(int(len(x_pos)/2) + 3)], color='#3151c4', marker='o', linewidth=0, markersize=2)
    ax.plot(x_pos[int(len(x_pos)/2 + 3):], y_pos[(int(len(x_pos)/2) + 3):], color='#bab634', marker='o', linewidth=0, markersize=2)

    a = 100
    b = 300
    c = 500
    d = 700
    e = 1000

    ax.plot(x_trj[:a], y_trj[:a], color="#a61a05", linewidth=4)
    ax.text(15, -7, 'A', color="#a61a05")

    ax.plot(x_trj[a:b], y_trj[a:b], color='#fcb103', linewidth=4)
    ax.text(43, -30, 'B', color='#fcb103')
    
    ax.plot(x_trj[b:c], y_trj[b:c], color="#28dae0", linewidth=4)
    ax.text(0, -35, 'C', color="#28dae0")

    ax.plot(x_trj[c:d], y_trj[c:d], color="#25ba1a", linewidth=4)
    ax.text(18, -65, 'D', color="#25ba1a")
    
    ax.plot(x_trj[d:e], y_trj[d:e], color="#1a3aba", linewidth=4)
    ax.text(-21, -54, 'E', color="#1a3aba")

    ax.plot(x_trj[e:], y_trj[e:], color="#a61a05", linewidth=4)


    # fig, axs = plt.subplots(2, 1, sharex=True, sharey=True)
    # points = np.array([x_car, y_car]).T.reshape(-1, 1, 2)

    # segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # norm = plt.Normalize(min(vel), max(vel))
    # lc = LineCollection(segments, cmap='viridis', norm=norm)
    # # Set the values used for colormapping
    # lc.set_array(vel)
    # lc.set_linewidth(2)
    # line = axs[0].add_collection(lc)
    # fig.colorbar(line, ax=axs[0])






    # ax.plot(x_car, y_car, "#c94b4b", linewidth=1)

    plt.tight_layout()
    ax.axis('scaled')  

    # ax.set_xlabel('X[m]')
    # ax.set_ylabel('Y[m]')
    # axs[0].set_xlim(.min(), x.max())
    # axs[0].set_ylim(-1.1, 1.1)
    # plt.show()    
    plt.savefig('mid_trj_speed.pgf', bbox_inches = 'tight')
    plt.savefig('mid_trj_speed.png',dpi=500, bbox_inches = 'tight')


if __name__ == '__main__':

    x_pos, y_pos = load_map()

    x_car, y_car = load_car()
    x_car_dt, y_car_dt = load_car_dt()

    vel = []
    for i in range(len(x_car_dt)):
        vel.append(math.hypot(x_car_dt[i],y_car_dt[i]))

    x_trj, y_trj = load_trj()
    # x_trj, y_trj = 1,1

    plot(x_pos, y_pos, x_car, y_car, x_trj, y_trj, vel)
