import MEAutility as MEA
from sklearn.decomposition import PCA
import numpy as np
import pandas as pd
from voxcell.nexus.voxelbrain import Atlas
import bluepysnap as bp
import sys


if __name__=='__main__':

    electrode_csv = sys.argv[1]
    numContacts = int(sys.argv[2])


    electrodePositions = np.zeros((numContacts,3))

    regionList = []
    layerList = []
    for i in range(numContacts):
        regionList.append('Outside')
        layerList.append('Outside')

    electrodeData = pd.DataFrame(data=electrodePositions,columns=['x','y','z'])

    layerData = pd.DataFrame(data=layerList,columns=['layer'])

    regionData = pd.DataFrame(data=regionList,columns=['region'])

    data = pd.concat((electrodeData,layerData),axis=1)
    data = pd.concat((data,regionData),axis=1)

    data.to_csv(electrode_csv)
