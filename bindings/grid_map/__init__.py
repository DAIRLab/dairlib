from .grid_map import *

class GridMap(GridMapBinding):
    def __setitem__(self, layer, val):
        self[layer][:] = val

    def __repr__(self):
        return f'<{self.getLength()[0]:.2f}x{self.getLength()[1]:.2f}x{self.getResolution():.2f} grid on {self.getLayers()} at {self.getPosition()}>'
