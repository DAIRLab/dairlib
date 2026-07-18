import numpy as np
import matplotlib
from matplotlib.collections import LineCollection
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch


class PlotStyler():
  @staticmethod
  def set_paper_styling():
    # matplotlib.rcParams['figure.figsize'] = 20, 12
    # matplotlib.rcParams['figure.figsize'] = 20, 6
    matplotlib.rcParams['figure.figsize'] = 16, 12
    plt.rcParams['figure.dpi'] = 200
    matplotlib.rcParams['figure.autolayout'] = True
    font = {'size': 20}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 4
    plt.set_cmap('tab20')
  @staticmethod
  def set_compact_styling():
    matplotlib.rcParams['figure.figsize'] = 20, 12
    font_size = 6
    plt.rc('legend', fontsize=font_size)
    plt.rc('axes', labelsize=font_size, titlesize=font_size)
    plt.rc('xtick', labelsize=font_size)
    plt.rc('ytick', labelsize=font_size)
    plt.rcParams['figure.dpi'] = 75
    matplotlib.rcParams['figure.autolayout'] = False
    font = {'size': font_size}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 2
    plt.set_cmap('tab20')

  def __init__(self, figure=None, nrows=1, ncols=1):

    # plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)  # List is [left, bottom, width, height]
    # self.cmap = plt.get_cmap('tab10')
    self.cmap = plt.get_cmap('tab20')
    self.blue = '#011F5B'
    self.red = '#990000'
    self.yellow = '#F2C100'
    self.grey = '#909090'
    self.orange = '#FE7F0E'
    # self.directory = None
    self.dpi = plt.rcParams['figure.dpi']
    self.directory = '/home/yangwill/Pictures/plot_styler/'
    plt.rc('legend', fontsize=24)
    plt.rc('axes', labelsize=24, titlesize=24)
    plt.rc('xtick', labelsize=24)
    plt.rc('ytick', labelsize=24)
    matplotlib.rcParams['figure.figsize'] = 12, 7
    matplotlib.rcParams['figure.autolayout'] = True
    matplotlib.rcParams['axes.xmargin'] = 0
    matplotlib.rcParams['axes.ymargin'] = 0
    # matplotlib.rcParams['toolbar'] = 'None'
    matplotlib.rcParams['text.latex.preamble'] = r"\usepackage{amsmath}"

    if figure is None:
      self.fig, self.axes = plt.subplots(nrows=nrows, ncols=ncols,
                                         sharex='all', dpi=self.dpi)
    else:
      self.fig = figure
      self.axes = figure.get_axes()
    if not isinstance(self.axes, np.ndarray):
      self.axes = [self.axes]
    # self.fig.add_axes([0.1, 0.15, 0.85, 0.75])  # List is [left, bottom, width, height]
    self.fig_id = self.fig.number
    plt.subplots_adjust(left=0.1, right=0.85, bottom=0.15, top=0.75)  # List is [left, bottom, width, height]
    return

  def attach(self):
    plt.figure(self.fig_id)

  def plot(self, xdata, ydata, xlim=None, ylim=None, color=None,
           linestyle=None,
           grid=False, xlabel=None, ylabel=None, title=None, legend=None,
           data_label=None, subplot_index=0):
    self.axes[subplot_index].plot(xdata, ydata, color=color,
                                  linestyle=linestyle, label=data_label)
    if xlim:
      self.axes[subplot_index].set_xlim(xlim)
    if ylim:
      self.axes[subplot_index].set_ylim(ylim)
    if xlabel:
      self.axes[subplot_index].set_xlabel(xlabel)
    if ylabel:
      self.axes[subplot_index].set_ylabel(ylabel)
    if title:
      self.axes[subplot_index].set_title(title)
    if legend:
      self.axes[subplot_index].legend(legend)
    self.axes[subplot_index].grid(grid, which='major')
    self.axes[subplot_index].autoscale()

  def plot_bands(self, x_low, x_high, y_low, y_high, color='C0',
                 subplot_index=0):
    plt.figure(self.fig_id)
    vertices = np.block([[x_low, x_high[::-1]],
                         [y_low, y_high[::-1]]]).T
    codes = Path.LINETO * np.ones(len(vertices), dtype=Path.code_type)
    codes[0] = Path.MOVETO
    path = Path(vertices, codes)
    patch = PathPatch(path, facecolor=color, edgecolor='none', alpha=0.3)
    self.axes[subplot_index].add_patch(patch)

  def show_fig(self):
    self.fig.show()
    return

  def save_fig(self, filename):
    self.fig.savefig(self.directory + filename, dpi=400, bbox_inches='tight')
    return

  def add_legend(self, labels, loc=0, subplot_index=0):
    legend = self.axes[subplot_index].legend(labels, loc=loc)
    self.axes[subplot_index].add_artist(legend)
    return

  def annotate(self, text, x, y, x_text, y_text, arrowprops=None, subplot_index=0):
    if not arrowprops:
      arrowprops = dict(facecolor='black')  # arrowstyle='->'
    self.axes[subplot_index].annotate(text, xy=(x, y), xytext=(
      x_text, y_text), arrowprops=arrowprops)

  def set_subplot_options(self, sharex=None, sharey=None):
    if sharex is not None:
      plt.setp(self.axes, sharex=sharex)
    if sharey is not None:
      plt.setp(self.axes, sharex=sharey)

  def tight_layout(self):
    # self.axes[0].autoscale(enable=True, axis='y', tight=True)
    self.axes[0].autoscale(enable=True, axis='x', tight=True)
