# SPDX-License-Identifier: GPL-3.0-or-later
import bluepysnap as bp
import json
import numpy as np
from voxcell.nexus.voxelbrain import Atlas
from sklearn.decomposition import PCA

def process_writeH5_inputs(inputs):

    path_to_fields = None 
    sigma = [0.277] # Default conductance, in S/m
    objective_csd_array_indices = None # Indices of electrodes to be used f

    if len(inputs)>6: # Specify conductance for analytic electrodes, or a potential field for reciprocity electrodes, or radius for spherical/disk objective csd, or electrode subsampling for disc objective csd. 

        for inputElement in inputs[6:]:
        
            inputType, inputList = splitInput(inputElement)
    
            if inputType == 'conductance':
                sigma = inputList
            elif inputType == 'ObjectiveCSD_Idx':
                objective_csd_array_indices = inputList
            else:
                path_to_fields = inputList

    return sigma, path_to_fields, objective_csd_array_indices

def process_inputList(inputList):

    '''
    Takes a list of inputs. If they are conductances, this function converts them to floats. Raises an error if some elements of the list  are floats and others are not
    '''

    inputType = None
    
    if inputList[0] == 'objective_csd_array_indices':
        inputType = 'ObjectiveCSD_Idx'
        inputList = inputList[1:]

    numFloats = 0

    for i, entry in enumerate(inputList):

        try: # If entry is a number, converts to float
            inputList[i] = float(entry)
            numFloats += 1
        except:
            pass

    if numFloats == len(inputList):
        if inputType == 'ObjectiveCSD_Idx':
            raise AssertionError('Indices for objective CSD arrays should be given in the form of a range, like \'5:9\', not as an integer')
        else:
            inputType = 'conductance'

    elif numFloats == 0:
        if inputType is None:
            inputType = 'paths'
        elif inputType == 'ObjectiveCSD_Idx':
            pass
    else:
        raise AssertionError('Mix of numbers and strings in input')

    return inputType, inputList

def splitInput(inputString):

    '''
    Takes an input string for conductance (for analytic electrodes) or paths to h5 potential field files (for reciprocity electrodes). If the string has multiple entries separated by spaces, returns a list of the entries. BlueRecording will then assume that each analytic or recprocity electrode uses a different conductance or E field, respecitively
    '''

    if ' ' in inputString: # If multiple potential field files or conductances, splits them into a list
        inputList = inputString.split(' ')
    else:
        inputList = [inputString] # Converts to list so that we can still call inputString[0]

    inputType, inputList = process_inputList(inputList)

    return inputType, inputList

def processSubsampling(inputString):
    '''
    Takes an input string of the form 5:9 specicifying the start and end points of an electrode array (i.e., with a particular spacing) and converts to integers
    '''

    start, end = inputString.split(':')

    return np.arange(int(start), int(end))

def getSimulationInfo(path_to_simconfig):

    '''
    Returns the following:
    report: Sonata report object
    node_ids: list of ids for which segment coefficients will be written
    '''
    
    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]] # We assume that the compartment report is the only report produced by the simulation

    population_name = getPopulationName(path_to_simconfig)

    report = r[population_name]
    
    nodeIds = report.node_ids

    return report, nodeIds

def getPopulationObject(path_to_simconfig):

    '''
    Returns the following:
    'population': SONATA population object
    '''
    
    rSim = bp.Simulation(path_to_simconfig)

    population_name = getPopulationName(path_to_simconfig)

    population = rSim.circuit.nodes[population_name]

    return population


def getPopulationName(path_to_simconfig):

    rSim = bp.Simulation(path_to_simconfig)
    r = rSim.reports[list(rSim.reports.keys())[0]] # We assume that the compartment report is the only report produced by the simulation

    population_name = r.population_names[0]

    return population_name

def getCircuitPath(path_to_simconfig):

    '''
    circuit: Path to the circuit used to generate the time steps. Gets written to the h5 file and is checked by neurodamus when and LFP simulation is run. LFP simulation will fail if it uses a different circuit than the one in the h5 file
    '''

    with open(path_to_simconfig) as f:

        circuitpath = json.load(f)['network']

    return circuitpath

def getMinimalReport(report,node_ids):

    '''
    Returns the following:
    data: dataframe with a compartment report, whose columns are the node_id and sectionId of each neuron
    '''


    data = report.get(group=node_ids,t_start=0,t_stop=report.frame_report.dt)
    
    data.columns = data.columns.rename('id',level=0)
    data.columns = data.columns.rename('section',level=1)

    return data

def getAtlasInfo(path_to_simconfig,electrodePositions):

    '''
    For an array of electrode positions, returns brain region and layer in which each electrode is located. 
    '''
    
    circuitpath = getCircuitPath(path_to_simconfig)

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
    
    population = getPopulationObject(path_to_simconfig)

    somaPos = population.get(properties=['x','y','z'],group=target) # Gets soma position

    center = np.mean(somaPos,axis=0).values

    pca = PCA(n_components=3)
    pca.fit(somaPos)
    main_axis = pca.components_[0]

    elevation = np.arctan2(np.sqrt(main_axis[0]**2+main_axis[1]**2),main_axis[2])
    azimuth = np.arctan2(main_axis[1],main_axis[0])

    return center, azimuth, elevation
    
