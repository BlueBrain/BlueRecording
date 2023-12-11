import pytest
import pandas as pd
import bluepy as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from writeH5_prelim import *


@pytest.fixture
def secCounts():

    '''
    Defines a data frame containing gids and section ids, equivalent to teh column indices from a voltage report
    '''

    sec_counts = pd.DataFrame(data=np.array([[1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2],[0,1,1,1,1,1,2,2,2,2,2,3,3,3,0,1,1,1,1,1]]).T,columns=('gid',bp.Section.ID))
    
    return sec_counts


@pytest.fixture
def h5File(tmp_path):

    filename = tmp_path / 'testfile.h5'

    file = h5py.File(filename,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[5,2,1],[7,3,0]]) # One soma with 1 3d point, 2 axon sections with4 and 3 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1,.3],[0,0,2,.3],[0,0,3,1],[0,0,3,1],[0,1,3,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(filename)


def test_makeElectrodeDict():

    csv = '/gpfs/bbp.cscs.ch/project/proj83/tharayil/generationCode/create_lfp_weights_for_neurodamus/tests/data/electrode.csv'
    expected = {'name':{'position':np.array([1,2,3]),'type':'EEG','region':'Outside','layer':'Outside'}}

    
    np.testing.assert_equal(makeElectrodeDict(csv,'EEG')['name'], expected['name'])


def test_ElectrodeFileStructure(tmp_path):

    outputfile = tmp_path / 'testfile.h5'

    h5file = h5py.File(outputfile,'w')

    gids = [1]

    electrodes = {'name':{'position':np.array([1,2,3]),'type':'EEG','region':'Outside','layer':'Outside'}}

    h5 = ElectrodeFileStructure(h5file, gids, electrodes, circuit='test') # Initializes fields in h5 file

    h5file.close()

    newFile = h5py.File(outputfile,'r')

    for key, value in electrodes['name'].items():

        if key == 'position':
            np.testing.assert_equal(newFile['electrodes/name/'+key][:], value)
        else: 
            np.testing.assert_equal(newFile['electrodes/name/'+key][()].decode(), value)

    np.testing.assert_equal(newFile['neuron_ids'][:],gids)

def test_write_neuron(tmp_path,secCounts):
    
    outputfile = tmp_path / 'testfile.h5'
    h5file = h5py.File(outputfile,'w')
    gids = [1]
    gid = gids[0]
    electrodes = {'name':{'position':np.array([1,2,3]),'type':'EEG','region':'Outside','layer':'Outside'}}
    h5 = ElectrodeFileStructure(h5file, gids, electrodes, circuit='test') # Initializes fields in h5 file
    secIds = pd.DataFrame(data=secCounts.values[:,1],index=secCounts.values[:,0]) # Data frame containing section ids, with gids as index

    write_neuron(h5, h5file, gid, electrodes,secIds.loc[gid])
    
    h5file.close()

    newFile = h5py.File(outputfile,'r')

    np.testing.assert_equal( newFile['electrodes/electrode_grid/1'][:],np.ones((14,2)) )
