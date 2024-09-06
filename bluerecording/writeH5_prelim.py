# SPDX-License-Identifier: GPL-3.0-or-later
import numpy as np
import h5py
import os
import pandas as pd
import sys
import bluepysnap as bp
import json
from .utils import *
import pkg_resources
import datetime

class ElectrodeFileStructure(object):

    '''
    This class writes datasets to the h5 file
    '''

    def __init__(self, h5, lst_ids, electrodes, population_name, **kwargs):

        '''
        h5: h5 file returned by h5py.File(filename,'w')
        lst_ids: node ids
        electrodes: Dictionary with metadata about electrodes
        population_name: Sonata population
        **kwargs: currently expected to take the circuit path, bluerecording version number, and date of writing
        '''


        dset = h5.create_dataset(population_name+"/node_ids", data=sorted(lst_ids))

        for k, v in kwargs.items():
            dset.attrs.create(k, v)

        index = 0

        ### Iterates through electrode dictionary to write metadata
        for key, electrode in electrodes.items(): # Iterates through electrodes

            h5.create_dataset("electrodes/" + str(key) + '/'+population_name,data=index) # Index of the column corresponding to this electrode in /electrodes/{population_name}/scaling_factors
            index += 1

            for item in electrode.items(): # Iterates through metadata fields for each electrode

                if item[0] == 'type' and isinstance(item[1],dict): # If electrodetype is a dict produced by process_ObjectiveCSD(), write the real electrode type string and the metadata

                    dset = h5.create_dataset("electrodes/" + str(key) + '/' + item[0],
                                      data=item[1]['type'])

                    for entry in item[1].items(): # Write parameters for objective calculation

                        if entry[0] !='type':
                            dset.attrs.create(entry[0],entry[1])

                else:

                    h5.create_dataset("electrodes/" + str(key) + '/' + item[0],
                              data=item[1])
        ####

        self._ids = np.array(lst_ids)

    def file(self):
        return h5py.File(self._fn, "r+")

    def lengths(self, gid):
        if gid not in self._ids:
            raise AssertionError("gid not present")

        return "lengths/" + str(int(gid))

    def offsets(self,population_name):
        return population_name+"/offsets"

    def weights(self, population_name):

        return '/electrodes/'+population_name+'/scaling_factors'

def get_offsets(sectionIdsFrame):

    unique, counts = np.unique(sectionIdsFrame['id'].values,return_counts=True) # Unique node_ids and number of segments per node id

    out_offsets = np.hstack((np.array([0]),np.cumsum(counts))) # Offset from start of list for each node id

    return out_offsets

def write_all_neuron(sectionIdsFrame, population_name, h5, file, electrode_struc):

    file.create_dataset(h5.weights(population_name), data=np.ones([len(sectionIdsFrame['id'].values),len(electrode_struc.items())+1])) # Initializes /electrodes/{population_name}/scaling_factors with array of ones of size nSegments x (nElectrodes+1)

    out_offsets = get_offsets(sectionIdsFrame)

    file.create_dataset(h5.offsets(population_name), data=out_offsets) # The offset for each node in the scaling_factors field


def makeElectrodeDict(electrode_csv):

    '''
    Reads electrode metadata from input csv file and writes it to a dictionary
    '''

    electrode_df = pd.read_csv(electrode_csv,header=0,index_col=0)

    electrodes = {}

    for i in range(len(electrode_df.values)): # Iterates through each electrode in array

        name = electrode_df.index.values[i]

        position = np.array([electrode_df['x'].iloc[i],electrode_df['y'].iloc[i],electrode_df['z'].iloc[i]])

        if 'layer' in electrode_df.columns:

            layer = electrode_df['layer'].iloc[i]

        else:

            layer = "NA"

        if 'region' in electrode_df.columns:
            region = electrode_df['region'].iloc[i]
        else:
            region = 'NA'

        if 'type' in electrode_df.columns:
            electrodeType = electrode_df['type'].iloc[i]

            if 'ObjectiveCSD' in electrodeType:

                electrodeType = process_objectiveCSD(electrodeType) # Returns a dict containing parameters for objective CSD electrodes

        else:
            electrodeType = 'LineSource'

        electrodes[name] = {'position': position,'type': electrodeType,
        'region':region,'layer':layer}


    return electrodes

def check_input_type_objectiveCSD(objectiveType,input):

    if objectiveType == 'ObjectiveCSD_Sphere' or objectiveType == 'ObjectiveCSD_Plane':
        try:
            assert len(input) == 3
        except:
            raise ValueError(objectiveType + ' must provide either no numerical parameters or exactly one')
    elif objectiveType =='ObjectiveCSD_Disk':
        try:
            assert len(input) == 3 or len(input)==4
        except:
            raise ValueError(objectiveType + ' must provide one or two numerical parameters')
    else:
        raise ValueError('Invalid electrode type')

    for numericalParameter in input[2:]:
        try:
            float(numericalParameter)
        except:
            raise ValueError('Invalid numerical parameter provided to objective CSD electrode')

    return 0

def process_objectiveCSD(electrodeType):

    '''
    If the electrode is one of the objective CSD electrodes, this function processes the input string to determine the radius (for a sphere) or thickness (for disk and plane) and diameter (for disk)
    If these items are provided, returns them in a dict
    If these items aren't provided, then returns the electrode type as a string

    electrodeType is of format objectveCSD_CalculationMethod, objectveCSD_CalculationMethod_X, or objectveCSD_Disk_X_Y
    if calculationMethod == Sphere, X == radius, in um
    If calculationMethod == Plane, X == thickness, in um
    if calculationMethod == Disk, X == radius, Y == thickess, both in um
    '''


    input = electrodeType.split('_')

    if len(input) < 2:
        raise ValueError(electrodeType + ' is an invalid objective electrode type')

    elif len(input)==2: # If no other options are provided, returns string, for backwards compatibility with previous versions
        return electrodeType

    else:

        objectiveType = input[0] + '_' + input[1]
        objectiveDict = {'type':objectiveType}

        check_input_type_objectiveCSD(objectiveType, input)

        if objectiveType == 'ObjectiveCSD_Sphere':

            radius = float(input[2])
            objectiveDict['radius'] = radius

        elif objectiveType == 'ObjectiveCSD_Plane':

            thickness = float(input[2])
            objectiveDict['thickness'] = thickness

        elif objectiveType == 'ObjectiveCSD_Disk':

            radius = float(input[2])
            objectiveDict['radius'] = radius

            if len(input)==4:
                thickness = float(input[3])
                objectiveDict['thickness'] = thickness
        else:
            raise ValueError("Invalid electrode type value")

        return objectiveDict

def initializeH5File(path_to_simconfig,outputfile,electrode_csv):

    '''
    path_to_simconfig refers to the simulation_config from the 1-timestep simulation used to get the segment positions
    electrode_csv is a csv file containing the position, region, and layer of each electrode
    type is either EEG or LFP
    '''

    report, nodeIds = getSimulationInfo(path_to_simconfig)

    population_name = getPopulationName(path_to_simconfig)

    circuitpath = getCircuitPath(path_to_simconfig)

    data = getMinimalReport(report, nodeIds)


    sectionIdsFrame = data.columns.to_frame()
    sectionIdsFrame.index = range(len(sectionIdsFrame))

    electrodes = makeElectrodeDict(electrode_csv) # Dictionary containing metadata about the electrodes

    h5file = h5py.File(outputfile,'w') # Creates h5 file for coefficients

    ### This block sets memory parameters that make writing the H5 file faster
    h5id = h5file.id
    cc = h5id.get_mdc_config()
    cc.max_size = 1024*1024*124
    h5id.set_mdc_config(cc)
    #####

    h5 = ElectrodeFileStructure(h5file, nodeIds, electrodes, population_name, circuit=circuitpath,version=pkg_resources.get_distribution("bluerecording").version,date=str(datetime.date.today())) # Initializes fields in h5 file


    write_all_neuron(sectionIdsFrame, population_name, h5, h5file, electrodes)  # For each node_id, initializes coefficient field in h5 file

    h5file.close()
