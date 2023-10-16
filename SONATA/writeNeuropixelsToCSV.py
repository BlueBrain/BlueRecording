import MEAutility as MEA
from sklearn.decomposition import PCA
import numpy as np
import pandas as pd
from voxcell.nexus.voxelbrain import Atlas
import sys

def getAtlasInfo(BlueConfig,electrodePositions):


    bluefile = open(BlueConfig,'r')
    bluelines = bluefile.readlines()
    bluefile.close()

    for line in bluelines:
        if 'Atlas' in line:
            atlasName = line.split('Atlas ')[-1].split('\n')[0]
            break

    atlas = Atlas.open(atlasName)
    brain_regions = atlas.load_data('brain_regions')

    region_map = atlas.load_region_map()

    regionList = []
    layerList = []


    for position in electrodePositions:

        try:

            for id_ in brain_regions.lookup([position]):

                region = region_map.get(id_, 'acronym')
                regionList.append(region.split(';')[0])
                layerList.append(region.split(';')[1])

        except:

            regionList.append('Outside')
            layerList.append('Outside')

    return regionList, layerList

def repositionElectrode(probe, path_to_Blueconfig):

    '''
    Aligns probe with center of cortical column
    '''

    c = bp.Circuit(path_to_Blueconfig)
    somaPos = c.cells.get({'$target': 'hex0'},properties=[bp.Cell.X, bp.Cell.Y, bp.Cell.Z])
    center = np.mean(somaPos,axis=0).values


    pca = PCA(n_components=3)
    pca.fit(somaPos)
    main_axis = pca.components_[0]

    elevation = np.arctan2(np.sqrt(main_axis[0]**2+main_axis[1]**2),main_axis[2])
    azimuth = np.arctan2(main_axis[1],main_axis[0])

    probe.rotate([0,1,0],elevation*180/np.pi)
    probe.rotate([0,0,1],azimuth*180/np.pi)
    probe.move(center)

    return(probe)

if __name__=='__main__':

    probe_name = sys.argv[1]
    path_to_Blueconfig = sys.argv[2]
    electrode_csv = sys.argv[3]

    probe = MEA.return_mea(probe_name)

    repositionElectrode(probe, path_to_Blueconfig)

    electrodePositions = probe.positions

    regionList, layerList = getAtlasInfo(path_to_Blueconfig, electrodePositions)

    electrodeData = pd.DataFrame(data=electrodePositions,columns=['x','y','z'])

    layerData = pd.DataFrame(data=layerList,columns=['layer'])

    data = pd.concat((electrodeData,layerData),axis=1)

    data.to_csv(electrode_csv)
