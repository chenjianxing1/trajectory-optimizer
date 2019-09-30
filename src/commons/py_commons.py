import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
import matplotlib as mpl

def DrawPolygon(pts, color="red"):
  plt.plot(pts[:, 0], pts[:, 1], color=color)

def GetColorMap(vmin=0., vmax=1., colormap='Blues'):
  cmap = plt.cm.get_cmap(colormap) # ListedColormap(['r', 'g', 'b'])
  norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
  return cmap, norm
