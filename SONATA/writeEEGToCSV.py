import MEAutility as MEA
from sklearn.decomposition import PCA
import numpy as np
import pandas as pd
from voxcell.nexus.voxelbrain import Atlas
import bluepysnap as bp
import sys


if __name__=='__main__':

    probe_name = sys.argv[1]
    path_to_simconfig = sys.argv[2]
    path_to_atlas = sys.argv[3]
    electrode_csv = sys.argv[4]


    electrodePositions = np.array([0,0,0]).reshape(1,3)

    regionList = ['Outside']
    layerList = ['Outside']

    electrodeData = pd.DataFrame(data=electrodePositions,columns=['x','y','z'])

    layerData = pd.DataFrame(data=layerList,columns=['layer'])

    regionData = pd.DataFrame(data=regionList,columns=['region'])

    data = pd.concat((electrodeData,layerData),axis=1)
    data = pd.concat((data,regionData),axis=1)

    data.to_csv(electrode_csv)
