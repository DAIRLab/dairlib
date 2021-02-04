import numpy as np
import matplotlib
from matplotlib.collections import LineCollection
import matplotlib.lines as mlines
import matplotlib.pyplot as plt

class PlotStyler():

  def set_default_styling(self, directory=None):

    matplotlib.rcParams["savefig.directory"] = directory
    matplotlib.rcParams['text.latex.preamble'] = [r"\usepackage{amsmath}"]
    font = {'size'   : 18}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 4

  def plot(self, xdata, ydata, xlim=None, ylim=None,
           grid=True, xlabel=None, ylabel=None, title=None, legend=None,
           conditional=False, col_conditional=None, s=None):

    if xlim:
      plt.xlim(xlim)
    if ylim:
      plt.ylim(ylim)
    if xlabel:
      plt.xlabel(xlabel, fontweight="bold")
    if ylabel:
      plt.ylabel(ylabel, fontweight="bold")
    if title:
      plt.title(title, fontweight="bold")
    plt.grid(grid, which='major')

  def showfig(self):
    plt.show()
    return

  def savefig(self, filename):
    plt.savefig(filename)
    return

  def annotate(self, text, x, y, x_text, y_text, arrowprops=None):
    ax = plt.gca()
    if not arrowprops:
      arrowprops = dict(facecolor='black')  # arrowstyle='->'
    ax.annotate(text, xy=(x, y), xytext=(
      x_text, y_text), arrowprops=arrowprops)