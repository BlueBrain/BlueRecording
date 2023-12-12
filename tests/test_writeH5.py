import pytest
import pandas as pd
import bluepy as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from writeH5 import *
from writeH5_prelim import ElectrodeFileStructure

@pytest.fixture
def write_ElectrodeFileStructure(tmp_path):
    
    outputfile = tmp_path / 'testfile.h5'

    h5file = h5py.File(outputfile,'w')

    gids = [1]

    electrodes = {'name':{'position':np.array([1,2,3]),'type':'EEG','region':'Outside','layer':'Outside'}}

    h5 = ElectrodeFileStructure(h5file, gids, electrodes, circuit='test') # Initializes fields in h5 file

    h5file.close()
    
    return outputfile

def test_getElectrodePositions(write_ElectrodeFileStructure):
    
    outputfile = write_ElectrodeFileStructure
    
    electrodePositions = getElectrodePositions(outputfile)
    
    expectedPositions = np.array([[1,2,3]])
    
    np.testing.assert_equal(electrodePositions,expectedPositions)