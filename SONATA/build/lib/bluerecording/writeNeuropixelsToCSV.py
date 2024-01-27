import MEAutility as MEA
from sklearn.decomposition import PCA
import numpy as np
import pandas as pd
from voxcell.nexus.voxelbrain import Atlas
import bluepysnap as bp
import sys

def getAtlasInfo(path_to_atlas,electrodePositions):

    atlas = Atlas.open(path_to_atlas)
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

def repositionElectrode(probe, path_to_simconfig):

    '''
    Aligns probe with center of cortical column
    '''

    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]] # We assume that the compartment report is the only report produced by the simulation

    population_name = r.population_names[0]

    population = rSim.circuit.nodes[population_name]

    somaPos = population.get(properties=['x','y','z'],group='hex0') # Gets soma position

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
    path_to_simconfig = sys.argv[2]
    path_to_atlas = sys.argv[3]
    electrode_csv = sys.argv[4]

    probe = MEA.return_mea(probe_name)

    repositionElectrode(probe, path_to_simconfig)

    electrodePositions = probe.positions

    regionList, layerList = getAtlasInfo(path_to_atlas, electrodePositions)

    electrodeData = pd.DataFrame(data=electrodePositions,columns=['x','y','z'])
    
    electrodeTypeList = []
    for p in electrodePositions:
        electrodeTypeList.append('LineSource')

    layerData = pd.DataFrame(data=layerList,columns=['layer'])

    regionData = pd.DataFrame(data=regionList,columns=['region'])
    
    electrodeTypeData = pd.DataFrame(data=electrodeTypeList,columns=['type'])

    data = pd.concat((electrodeData,layerData),axis=1)
    data = pd.concat((data,regionData),axis=1)
    data = pd.concat((data,electrodeTypeData),axis=1)

    data.to_csv(electrode_csv)
