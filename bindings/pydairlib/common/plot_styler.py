import numpy as np
import matplotlib
from matplotlib.collections import LineCollection
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch


class PlotStyler():

  def __init__(self):
    # self.cmap = plt.get_cmap('tab10')
    self.cmap = plt.get_cmap('tab20')
    self.blue = '#011F5B'
    self.red = '#990000'
    self.yellow = '#F2C100'
    self.grey = '#909090'
    self.orange = '#FE7F0E'
    self.directory = None
    self.penn_color_wheel = [self.blue, self.red, self.yellow, self.grey, self.orange]
    return

  def set_default_styling(self, directory=None, figsize=None):
    self.directory = directory
    matplotlib.rcParams["savefig.directory"] = directory
    matplotlib.rcParams['text.latex.preamble'] = [r"\usepackage{amsmath}"]
    # matplotlib.rcParams['figure.figsize'] = 20, 12
    # matplotlib.rcParams['figure.figsize'] = 20, 6
    # matplotlib.rcParams['figure.figsize'] = 8, 5
    if (figsize == None):
      matplotlib.rcParams['figure.figsize'] = 8, 12
    else:
      matplotlib.rcParams['figure.figsize'] = figsize[0], figsize[1]
    matplotlib.rcParams['figure.autolayout'] = True
    font = {'size': 18}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 4
    plt.set_cmap('tab20')
    self.directory = directory

  def set_figsize(self, size):
    matplotlib.rcParams['figure.figsize'] = size

  def plot(self, xdata, ydata, xlim=None, ylim=None, color=None, linestyle=None,
           grid=True, xlabel=None, ylabel=None, title=None, legend=None, data_label=None):

    plt.plot(xdata, ydata, color=color, linestyle=linestyle, label=data_label)
    if xlim:
      plt.xlim(xlim)
    if ylim:
      plt.ylim(ylim)
    if xlabel:
      # plt.xlabel(xlabel, fontweight="bold")
      plt.xlabel(xlabel)
    if ylabel:
      # plt.ylabel(ylabel, fontweight="bold")
      plt.ylabel(ylabel)
    if title:
      # plt.title(title, fontweight="bold")
      plt.title(title)
    if legend:
      plt.legend(legend)

    plt.grid(grid, which='major')

  def scatter(self, xdata, ydata, xlim=None, ylim=None, color=None,
           grid=True, xlabel=None, ylabel=None, title=None, legend=None, data_label=None):

    plt.scatter(xdata, ydata, color=color, label=data_label)
    if xlim:
      plt.xlim(xlim)
    if ylim:
      plt.ylim(ylim)
    if xlabel:
      # plt.xlabel(xlabel, fontweight="bold")
      plt.xlabel(xlabel)
    if ylabel:
      # plt.ylabel(ylabel, fontweight="bold")
      plt.ylabel(ylabel)
    if title:
      # plt.title(title, fontweight="bold")
      plt.title(title)
    if legend:
      plt.legend(legend)

    plt.grid(grid, which='major')

  def plot_bands(self, x_low, x_high, y_low, y_high, color='C0'):
    vertices = np.block([[x_low, x_high[::-1]],
                         [y_low, y_high[::-1]]]).T
    codes = Path.LINETO * np.ones(len(vertices), dtype=Path.code_type)
    codes[0] = Path.MOVETO
    path = Path(vertices, codes)
    patch = PathPatch(path, facecolor=color, edgecolor='none', alpha=0.3)
    ax = plt.gca()
    # ax.plot(xdata, ydata)
    ax.add_patch(patch)

  def show_fig(self):
    plt.show()
    return

  def save_fig(self, filename):

    plt.savefig(self.directory + filename, dpi=200)
    plt.close()
    return

  def add_legend(self, legend, loc=0):
    plt.legend(legend, loc=loc)
    return

  def annotate(self, text, x, y, x_text, y_text, arrowprops=None):
    ax = plt.gca()
    if not arrowprops:
      arrowprops = dict(facecolor='black')  # arrowstyle='->'
    ax.annotate(text, xy=(x, y), xytext=(
      x_text, y_text), arrowprops=arrowprops)
