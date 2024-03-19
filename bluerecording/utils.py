import bluepysnap as bp
import json
import numpy as np
from voxcell.nexus.voxelbrain import Atlas
from sklearn.decomposition import PCA


def getSimulationInfo(path_to_simconfig):

    '''
    Returns the following:
    circuit: Path to the circuit used to generate the time steps. Gets written to the h5 file and is checked by neurodamus when and LFP simulation is run. LFP simulation will fail if it uses a different circuit than the one in the h5 file
    population_name: SONATA population name
    node_ids: list of ids for which segment coefficients will be written
    data: dataframe with a compartment report, whose columns are the node_id and sectionId of each neuron
    '''
    
    with open(path_to_simconfig) as f:

        circuitpath = json.load(f)['network']

    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]] # We assume that the compartment report is the only report produced by the simulation

    circuit = rSim.circuit

    population_name = r.population_names[0]

    report = r[population_name]
    
    nodeIds = report.node_ids


    data = report.get(group=nodeIds,t_start=0,t_stop=r.dt)
    
    data.columns = data.columns.rename('id',level=0)
    data.columns = data.columns.rename('section',level=1)

    population = rSim.circuit.nodes[population_name]

    return report, circuitpath, population, population_name, nodeIds, data

def getAtlasInfo(path_to_simconfig,electrodePositions):

    '''
    For an array of electrode positions, returns brain region and layer in which each electrode is located. 
    '''
    
    with open(path_to_simconfig) as f:
        circuitpath = json.load(f)['network']

    with open(circuitpath) as f:
        path_to_atlas = json.load(f)['components']['provenance']['atlas_dir']


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

def alignmentInfo(path_to_simconfig,target):

    '''
    Gets loction and angle information in order to align a probe with long axis of of the specified target (typically a cortical column)
    '''
    
    _, _, population, _, _, _ = getSimulationInfo(path_to_simconfig)

    somaPos = population.get(properties=['x','y','z'],group=target) # Gets soma position

    center = np.mean(somaPos,axis=0).values

    pca = PCA(n_components=3)
    pca.fit(somaPos)
    main_axis = pca.components_[0]

    elevation = np.arctan2(np.sqrt(main_axis[0]**2+main_axis[1]**2),main_axis[2])
    azimuth = np.arctan2(main_axis[1],main_axis[0])

    return center, azimuth, elevation
    
