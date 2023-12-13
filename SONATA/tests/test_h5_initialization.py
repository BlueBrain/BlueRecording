import pytest
import pandas as pd
import bluepysnap as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from writeH5_prelim import *


def test_makeElectrodeDict(electrodes):

    csv = '/gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/tests/data/electrode.csv'
    expected = electrodes

    
    np.testing.assert_equal(makeElectrodeDict(csv,'EEG')['name'], expected['name'])


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
    
def test_offset(secCounts):
    
    offsets = get_offsets(secCounts)
    expected_offsets = np.array([0,19])
    
    np.testing.assert_equal(offsets, expected_offsets)

def test_write_neuron(writeNeuron,population_name):
    
    '''
    Tests that weights are initialized correctly for a given neuron
    '''

    newFile = h5py.File(writeNeuron[0],'r') # writeNeuron is a fixture that calls write_neuron from writeH5_prelim.py with the appropriate arguments, and returns the path to the h5 file and the h5 file object

    np.testing.assert_equal( newFile['electrodes/'+population_name+'/scaling_factors'][:],np.ones((25,2)) )
    
    np.testing.assert_equal( newFile[population_name+'/offsets'][:],np.array([0,19]) )
    
    
