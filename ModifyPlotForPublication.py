#http://damon-is-a-geek.com/publication-ready-the-first-time-beautiful-reproducible-plots-with-matplotlib.html
from matplotlib import rcParams

def ModifyPlotForPublication():
    textSize = 8
    rcParams['axes.labelsize'] = textSize
    rcParams['xtick.labelsize'] = textSize
    rcParams['ytick.labelsize'] = textSize
    rcParams['legend.fontsize'] = textSize

    rcParams['font.family'] = 'serif'
    rcParams['font.serif'] = ['Computer Modern Roman']
    rcParams['text.usetex'] = True
