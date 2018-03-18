__author__ = 'dirkvangheel'

from matplotlib import pylab as plt

COLOR = {
    True:  '#6699cc',
    False: '#ffcc33'
}
def v_color(ob):
    return COLOR[ob.is_simple]

def plot_polygon(ax, p):
    x,y = p.exterior.coords.xy
    ax.plot(x, y, 'o', color='#999999', zorder=1)
    ax.plot(x, y, color=v_color(p), alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)


def plot_linestring(ax, l):
    x,y = l.xy
    ax.plot(x, y, 'o', color='#999999', zorder=1)
    ax.plot(x, y, color=v_color(l), alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)


from matplotlib import collections  as mc

def plot_polygon_map(polygons,
                     points={},
                     markers={},
                     paths={},
                     edges={},
                     ax=None, figsize=(10,10),
                     title="polygon map"):
    if ax is None:
        fig, ax = plt.subplots(1,1, figsize=figsize)

    # Note:
    # in the data North corresponds to the first dimension (x) and
    # East to the second dimension (y)
    # in plots, we will put North on the y-axis and East on the x-axis
    # adding these constants for use in indexing to avoid confusion
    N = 0
    E = 1

    obstacle_color="#999999"
    for p, h in polygons:
        north,east = p.exterior.coords.xy
        ax.fill(east, north, alpha=0.2, fc=obstacle_color, ec='none')

    for label in edges:
        color, d = edges[label]
        # put east on x-axis and north on y-axis
        lines = [((p1[E], p1[N]),(p2[E], p2[N])) for (p1, p2) in d]
        lc = mc.LineCollection(lines, colors=color, linewidths=2)
        ax.add_collection(lc)

    for label in points:
        color, p = points[label]
        ax.plot(p[:,E], p[:,N], 'o', c=color, label=label)

    for label in markers:
        color, p = markers[label]
        ax.plot(p[:,E], p[:,N], 'x', c=color, label=label, markersize=4)

    for label in paths:
        color, p = paths[label]
        ax.plot(p[:,E], p[:,N], '-', c=color, label=label)

    _ = ax.set_xlabel('EAST')
    _ = ax.set_ylabel('NORTH')
    _ = ax.set_title(title)
    _ = ax.legend(loc="best")
