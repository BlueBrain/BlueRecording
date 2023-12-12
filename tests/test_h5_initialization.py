import pytest
import pandas as pd
import bluepy as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from writeH5_prelim import *


def test_makeElectrodeDict(electrodes):

    csv = '/gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/tests/data/electrode.csv'
    expected = electrodes

    
    np.testing.assert_equal(makeElectrodeDict(csv,'EEG')['name'], expected['name'])


def test_ElectrodeFileStructure(write_ElectrodeFileStructure, electrodes, gids):
    
    '''
    Tests that electrode names and positions are written correctly
    '''

    outputfile, h5 = write_ElectrodeFileStructure

    newFile = h5py.File(outputfile,'r')

    for key, value in electrodes['name'].items():

        if key == 'position':
            np.testing.assert_equal(newFile['electrodes/name/'+key][:], value)
        else: 
            np.testing.assert_equal(newFile['electrodes/name/'+key][()].decode(), value)

    np.testing.assert_equal(newFile['neuron_ids'][:],gids)

def test_write_neuron(writeNeuron):
    
    '''
    Tests that weights are initialized correctly for a given neuron
    '''

    newFile = h5py.File(writeNeuron,'r')

    np.testing.assert_equal( newFile['electrodes/electrode_grid/1'][:],np.ones((19,2)) )