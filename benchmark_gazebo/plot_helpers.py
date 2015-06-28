import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams.update({'font.size': 16})
from csv_dictionary import *

boxes = makeCsvDictOfArrays('test_results/BENCHMARK_boxes_dt.csv')

color1 = [0, 0, 0.5]
color2 = [0.5, 0.5, 0.5]
linestyle1 = '-'
linestyle2 = '--'

def plotTimePosition3(time, position3):
    plt.gcf()
    plt.plot(time, position3[:,0], linewidth=4.0, linestyle=linestyle1, color=color1)
    plt.plot(time, position3[:,1], linewidth=4.0, linestyle=linestyle2, color=color1)
    plt.plot(time, position3[:,2], linewidth=2.0, linestyle=linestyle1, color=color2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.grid()
    plt.legend(['x','y','z'], loc='best');

# helper function for resizing axes
def vector_scale(x, scale):
    mean = np.mean(x)
    centered = x - mean
    return mean + centered*scale
def vector_log10_scale(x, scale):
    logx = np.log10(x)
    scaled = vector_scale(logx, scale)
    return [10**l for l in scaled]

# Create a plot with time step Dt on horizontal axis
# Value of `yname` plotted on vertical axis
def plotEnginesDt(params, yname
                , axscale=1.1
                , ayscale=1.1
                , csvDict=boxes
                , legend='best'
                , xname='dt'
                , xlabel='Time step (s)'
                , ylabel='Error'
                , yscale='linear'
                , title='title'
                , skipDart=False
                ):
    engines = {}
    engines['bullet'] = ['$B$', 'b--']
    if not skipDart:
        engines['dart'] = ['$d$', 'g--']
    engines['ode'] = ['$O$', 'r--']
    engines['simbody'] = ['$S$', 'k--']
    fig = plt.figure()
    xdata = {}
    ydata = {}
    for e in sorted(engines.keys()):
        params['engine'] = e
        ii = np.array(list(query(csvDict, params)))
        xdata[e] = csvDict[xname][ii]
        ydata[e] = csvDict[yname][ii]
        color = engines[e][1][0]
        plt.plot(xdata[e]
               , ydata[e]+np.finfo(float).eps
               , engines[e][1]
               , mfc=color
               , marker=engines[e][0]
               , markersize=20.0
               , markeredgecolor=color
               , linewidth=2.0
               )
    plt.grid()
    plt.xlabel(xlabel, fontsize=18)
    plt.ylabel(ylabel, fontsize=18)
    plt.gca().set_yscale(yscale)
    plt.title(title)
    plt.gcf().set_size_inches(10, 6)
    plt.xlim(vector_scale(plt.xlim(), axscale))
    if yscale == 'log':
        plt.ylim(vector_log10_scale(plt.ylim(), ayscale))
    else:
        plt.ylim(vector_scale(plt.ylim(), ayscale))
    plt.legend(sorted(engines.keys()), loc=legend)
    plt.show();
    # some extra info about each plot
    xdata_minmax = {}
    ydata_minmax = {}
    for e in sorted(engines.keys()):
        xdata_minmax[e] = [min(xdata[e]), max(xdata[e])]
        ydata_minmax[e] = [min(ydata[e]), max(ydata[e])]

def plotEnginesTime(params, yname
                  , csvDict=boxes
                  , legend='best'
                  , skipDart=False
                  , xname='timeRatio'
                  , xlabel='Time ratio (real / sim)'
                  , ylabel='Error'
                  , yscale='linear'
                  , title='title'
                   ):
    plotEnginesDt(params, yname
                  , csvDict=csvDict
                  , legend=legend
                  , skipDart=skipDart
                  , xname=xname
                  , xlabel=xlabel
                  , ylabel=ylabel
                  , yscale=yscale
                  , title=title
                  )
def plotEnginesModelCount(params, yname
                  , csvDict=boxes
                  , legend='best'
                  , skipDart=False
                  , xname='modelCount'
                  , xlabel='Model count'
                  , ylabel='Time ratio (real / sim)'
                  , yscale='linear'
                  , title='title'
                   ):
    plotEnginesDt(params, yname
                  , csvDict=csvDict
                  , legend=legend
                  , skipDart=skipDart
                  , xname=xname
                  , xlabel=xlabel
                  , ylabel=ylabel
                  , yscale=yscale
                  , title=title
                  )

def plot3TimeDt(params
                , csvDict=boxes
                , yname='linPositionErr_maxAbs'
                , title=''
                , skipDart=False
                , yscale='linear'
                ):
    plotEnginesDt(params
                , csvDict=csvDict
                , yname=yname
                , title=title
                , skipDart=skipDart
                , yscale=yscale
                )
    plotEnginesDt(params
                , csvDict=csvDict
                , yname='timeRatio'
                , ylabel='Computational time / sim time'
                , title='Computational time'
                , skipDart=skipDart
                , yscale=yscale
                )
    plotEnginesTime(params
                , csvDict=csvDict
                , yname=yname
                , title=title
                , skipDart=skipDart
                , yscale=yscale
                )

def plotErrorDt(classname, title_prefix
                , csvDict=boxes
                , legend='best'
                , yscale='linear'):
    p = {}
    p['classname'] = classname
    title_prefix = title_prefix
    plotEnginesDt(p, yname='linPositionErr_maxAbs', title=title_prefix + 'position'
                  , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesDt(p, yname='angPositionErr_mag_maxAbs', title=title_prefix + 'angle'
                  , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesDt(p, yname='linVelocityErr_maxAbs', title=title_prefix + 'velocity'
                  , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesDt(p, yname='angMomentumErr_maxAbs', title=title_prefix + 'angular momentum'
                  , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesDt(p, yname='energyError_maxAbs', title=title_prefix + 'energy'
                  , csvDict=csvDict, legend=legend, yscale=yscale)

def plotTimeDt(classname, title_prefix
                , csvDict=boxes
                , legend='best'
                , yscale='linear'):
    p = {}
    p['classname'] = classname
    title_prefix = title_prefix
    plotEnginesDt(p, yname='timeRatio', title=title_prefix + 'time ratio'
                  , ylabel='Time ratio (real / sim)'
                  , csvDict=csvDict, legend=legend, yscale=yscale)

def plotErrorTime(classname, title_prefix
                  , csvDict=boxes
                  , legend='best'
                  , yscale='linear'):
    p = {}
    p['classname'] = classname
    title_prefix = title_prefix
    plotEnginesTime(p, yname='linPositionErr_maxAbs', title=title_prefix + 'position'
                    , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesTime(p, yname='angPositionErr_mag_maxAbs', title=title_prefix + 'angle'
                    , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesTime(p, yname='linVelocityErr_maxAbs', title=title_prefix + 'velocity'
                    , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesTime(p, yname='angMomentumErr_maxAbs', title=title_prefix + 'angular momentum'
                    , csvDict=csvDict, legend=legend, yscale=yscale)
    plotEnginesTime(p, yname='energyError_maxAbs', title=title_prefix + 'energy'
                    , csvDict=csvDict, legend=legend, yscale=yscale)


