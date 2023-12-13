import pytest
import pandas as pd
import bluepysnap as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from writeH5 import *
from writeH5_prelim import ElectrodeFileStructure

@pytest.fixture
def filesPerFolder():
    return 50

@pytest.fixture
def numPositionFiles():
    return 100

@pytest.fixture
def electrodePosition():
    return np.array([10,10,10])

@pytest.fixture
def sigma():
    return 1

@pytest.fixture
def data_twoSections():
    
    '''
    Defines a data frame mimicking a voltage report, with columns containing gids and section ids
    '''
    
    columns = [[1,1],[0,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['gid',bp.Section.ID])

    data = pd.DataFrame(data=np.zeros([1,len(columns[0])]),columns=columnMultiIndex)
    
    return data

@pytest.fixture
def positions():
    
    '''
    Defines a position dataframe corresponding to the two-section neuron described above.
    Positions given for soma and for start and end of section
    '''
    
    columns = [[1,1,1],[0,1,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['gid',bp.Section.ID])
    
    positions = np.array([[0.,0.,0.],[0.,0.,0.],[0.,0.,1.]])

    pos = pd.DataFrame(data=positions,columns=columnMultiIndex)
    
    return pos

def test_get_position_file(filesPerFolder, numPositionFiles):
    
    rank = 0
    
    assert get_position_file(filesPerFolder,numPositionFiles,rank)=='0/positions0.pkl'
    
    rank = 453
    
    assert get_position_file(filesPerFolder,numPositionFiles,rank)=='1/positions53.pkl'

def test_get_indices(numPositionFiles):
    
    rank = 1
    nranks = 400
    
    iteration, iterationSize = get_indices(rank, nranks, numPositionFiles)
    
    assert iterationSize == 250
    assert iteration == 0
    
def test_getSegmentMidpts(positions,gids):
    
    columns = [[1,1],[0,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=[None,bp.Section.ID])
    
    expectedPos = np.array([[0.,0.,0.],[0.,0.,.5]])
    
    expectedPositions = pd.DataFrame(data=expectedPos.T,columns=columnMultiIndex)
    expectedPositions.index = range(len(expectedPositions))
    
    outputPos = getSegmentMidpts(positions,gids)
    
    
    pd.testing.assert_frame_equal(outputPos,expectedPositions)
    
def test_add_coeffs(writeNeuron,gids):
    
    h5File = writeNeuron[0]
    
    h5 = h5py.File(h5File,'r+')
    
    gid = gids[0]
    
    test_data = np.arange(19)[:,np.newaxis]
    
    add_data(h5,gid,test_data)
    
    expectedCoeffs = np.array([np.arange(19),np.ones(19)]).T
    
    np.testing.assert_equal( h5['electrodes/electrode_grid/1'][:], expectedCoeffs )
    
    h5.close()
    
def test_get_coeffs_lfp(positions,data_twoSections,electrodePosition,sigma):
    
    columns = data_twoSections.columns
    coeffs = get_coeffs_lfp(positions,columns,electrodePosition,sigma)
    
    somaDistance = np.sqrt(3*10**2)
    expectedSomaCoeff = 1/(4*np.pi*sigma*somaDistance)
    
    expectedLineCoeff = get_line_coeffs(np.array([0,0,0]),np.array([0,0,1]),electrodePosition,sigma)
    
    expectedOutput = pd.DataFrame(data=np.hstack((expectedSomaCoeff,expectedLineCoeff))[np.newaxis,:],columns=columns)
    
    pd.testing.assert_frame_equal(coeffs,expectedOutput)
    
def test_get_coeffs_eeg(positions,write_potentialField,gids):
    
    testPositions = getSegmentMidpts(positions,gids)
    potentials = get_coeffs_eeg(testPositions,write_potentialField)
    
    columns = [[1,1],[0,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=[None,bp.Section.ID])
    
    expectedPotential = pd.DataFrame(data=np.array([0,0.5e-6])[np.newaxis,:],columns=columnMultiIndex)
    
    pd.testing.assert_frame_equal(potentials,expectedPotential)
    
