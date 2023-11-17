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
    
    if len(sys.argv)>3: # If an electrode name string is provided, create a list of names, either containing all elements in the string, or repeating the string for as many electrodes as there are
        name = sys.argv[3]
        if ' ' in name:
            names = name.split(' ')
        else:
            names = [name]

        if len(names) != numContacts:
            
            if len(names) == 1: # If one name is provided for multiple contacts, each contact gets the name name_i
                names = []
                for i in range(numContacts):
                    names.append(name+'_'+str(i))
            else:
                raise ValueError('Number of names must be either 1 or equal to number of contacts')


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

    if len(sys.argv)>3:
        data.index = names

    
    data.to_csv(electrode_csv)
