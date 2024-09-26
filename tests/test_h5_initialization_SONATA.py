# SPDX-License-Identifier: GPL-3.0-or-later
import pytest
import pandas as pd
import bluepysnap as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from bluerecording.writeH5_prelim import *


def test_makeElectrodeDict(electrodes):

    csv = '/gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/tests/data/electrode.csv'
    expected = electrodes

    np.testing.assert_equal(makeElectrodeDict(csv)['name'], expected['name'])

def test_ElectrodeFileStructure(write_ElectrodeFileStructure, electrodes, gids,population_name):
    
    '''
    Tests that electrode names and positions are written correctly
    '''

    outputfile, h5 = write_ElectrodeFileStructure # write_ElectrodeFileStructure is a fixture that initializes ElectrodeFileStructure from writeH5_prelim.py with the appropriate parameters

    newFile = h5py.File(outputfile,'r')

    for key, value in electrodes['name'].items():
        
        if key == 'position':
            np.testing.assert_equal(newFile['electrodes/name/'+key][:], value)
        else: 
            np.testing.assert_equal(newFile['electrodes/name/'+key][()].decode(), value)
            
    np.testing.assert_equal(newFile[population_name+'/node_ids'][:],gids)
    
    np.testing.assert_equal(newFile[population_name+'/node_ids'].attrs['circuit'], 'test')


def test_ElectrodeFileStructure_objective(write_ElectrodeFileStructure_objective, electrodes_objective, gids, population_name):
    '''
    Tests that electrode names and positions are written correctly
    '''

    outputfile, h5 = write_ElectrodeFileStructure_objective  # write_ElectrodeFileStructure is a fixture that initializes ElectrodeFileStructure from writeH5_prelim.py with the appropriate parameters

    newFile = h5py.File(outputfile, 'r')

    for key, value in electrodes_objective['name'].items():

        if key == 'position':
            np.testing.assert_equal(newFile['electrodes/name/' + key][:], value)
        elif key == 'type':
            np.testing.assert_equal(newFile['electrodes/name/type'][()].decode(),value['type'])
            np.testing.assert_equal(newFile['electrodes/name/type'].attrs.get('radius'),value['radius'])
            np.testing.assert_equal(newFile['electrodes/name/type'].attrs.get('thickness'), value['thickness'])
        else:
            np.testing.assert_equal(newFile['electrodes/name/' + key][()].decode(), value)

    np.testing.assert_equal(newFile[population_name + '/node_ids'][:], gids)

    np.testing.assert_equal(newFile[population_name + '/node_ids'].attrs['circuit'], 'test')
    
def test_offset(secCounts):
    
    offsets = get_offsets(secCounts)
    expected_offsets = np.array([0,19,25])
    
    np.testing.assert_equal(offsets, expected_offsets)

def test_write_neuron(writeNeuron,population_name):
    
    '''
    Tests that weights are initialized correctly for a given neuron
    '''

    newFile = h5py.File(writeNeuron[0],'r') # writeNeuron is a fixture that calls write_neuron from writeH5_prelim.py with the appropriate arguments, and returns the path to the h5 file and the h5 file object

    np.testing.assert_equal( newFile['electrodes/'+population_name+'/scaling_factors'][:],np.ones((25,2)) )
    
    np.testing.assert_equal( newFile[population_name+'/offsets'][:],np.array([0,19,25]) )
    
def test_check_input_type_objectiveCSD():

    electrodeType = 'ObjectiveCSD_Sphere'
    electrode = 'ObjectiveCSD_Sphere_4'.split('_')
    assert check_input_type_objectiveCSD(electrodeType,electrode) == 0

    electrode = 'ObjectiveCSD_Sphere_4_2'.split('_')
    with pytest.raises(ValueError) as excinfo:
        check_input_type_objectiveCSD(electrodeType,electrode)
    assert str(excinfo.value) == 'ObjectiveCSD_Sphere must provide either no numerical parameters or exactly one'

    electrode = 'ObjectiveCSD_Sphere_twss'.split('_')
    with pytest.raises(ValueError) as excinfo:
        check_input_type_objectiveCSD(electrodeType,electrode)
    assert str(excinfo.value) == 'Invalid numerical parameter provided to objective CSD electrode'


    electrodeType = 'ObjectiveCSD_Plane'
    electrode = 'ObjectiveCSD_Plane_4'.split('_')
    assert check_input_type_objectiveCSD(electrodeType,electrode) == 0

    electrode = 'ObjectiveCSD_Plane_4_2'.split('_')
    with pytest.raises(ValueError) as excinfo:
        check_input_type_objectiveCSD(electrodeType,electrode)
    assert str(excinfo.value) == 'ObjectiveCSD_Plane must provide either no numerical parameters or exactly one'


    electrodeType = 'ObjectiveCSD_Disk'
    electrode = 'ObjectiveCSD_Disk_4'.split('_')
    assert check_input_type_objectiveCSD(electrodeType, electrode) == 0

    electrode = 'ObjectiveCSD_Disk_4.1_200'.split('_')
    assert check_input_type_objectiveCSD(electrodeType, electrode) == 0

    electrode = 'ObjectiveCSD_Disk_4.1_2as'.split('_')
    with pytest.raises(ValueError) as excinfo:
        check_input_type_objectiveCSD(electrodeType, electrode)
    assert str(excinfo.value) == 'Invalid numerical parameter provided to objective CSD electrode'

def test_process_objectiveCSD():

    with pytest.raises(ValueError) as excinfo:
        process_objectiveCSD('LineSource')
    assert str(excinfo.value) == 'LineSource is an invalid objective electrode type'

    output = process_objectiveCSD('ObjectiveCSD_Disk')
    assert output == 'ObjectiveCSD_Disk'

    output = process_objectiveCSD('ObjectiveCSD_Sphere_2')
    assert output == {'type':'ObjectiveCSD_Sphere','radius':2.0}

    output = process_objectiveCSD('ObjectiveCSD_Disk_2')
    assert output == {'type': 'ObjectiveCSD_Disk', 'radius': 2.0}

    output = process_objectiveCSD('ObjectiveCSD_Disk_2.1_44')
    assert output == {'type': 'ObjectiveCSD_Disk', 'radius': 2.1,'thickness':44.}

    output = process_objectiveCSD('ObjectiveCSD_Plane_44')
    assert output == {'type': 'ObjectiveCSD_Plane','thickness': 44.}


