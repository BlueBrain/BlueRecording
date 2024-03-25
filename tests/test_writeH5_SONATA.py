import pytest
import pandas as pd
import bluepysnap as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from bluerecording.writeH5 import *
from bluerecording.writeH5_prelim import ElectrodeFileStructure

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
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])

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
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])
    
    positions = np.array([[0.,0.,0.],[0.,0.,0.],[0.,0.,1.]])

    pos = pd.DataFrame(data=positions,columns=columnMultiIndex)
    
    return pos

def test_get_position_file_name(filesPerFolder, numPositionFiles):
    
    rank = 0
    
    assert get_position_file_name(filesPerFolder,numPositionFiles,rank)=='0/positions0.pkl'
    
    rank = 453
    
    assert get_position_file_name(filesPerFolder,numPositionFiles,rank)=='1/positions53.pkl'

def test_get_indices(numPositionFiles):
    
    rank = 1
    nranks = 400
    neuronsPerFile = 1000
    
    iteration, iterationSize = get_indices(rank, nranks, neuronsPerFile, numPositionFiles)
    
    assert iterationSize == 250
    assert iteration == 0

    with pytest.raises(AssertionError):
        nranks = 2
        iteration, iterationSize = get_indices(rank, nranks, neuronsPerFile, numPositionFiles)

    
def test_getSegmentMidpts(positions,gids):
    
    columns = [[1,1],[0,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])
    
    expectedPos = np.array([[0.,0.,0.],[0.,0.,.5]])
    
    expectedPositions = pd.DataFrame(data=expectedPos.T,columns=columnMultiIndex)
    expectedPositions.index = range(len(expectedPositions))
    
    outputPos = getSegmentMidpts(positions,gids)
    
    
    pd.testing.assert_frame_equal(outputPos,expectedPositions)
    
    
def test_add_coeffs(writeNeuron,gids,population_name,data):
        
    h5File = writeNeuron[0]
    
    h5 = h5py.File(h5File,'r+')
    
    test_data = pd.DataFrame(data=np.arange(25)[np.newaxis,:],columns=data.columns)
    
    add_data(h5,gids,test_data,population_name)
    
    expectedCoeffs = np.array([np.arange(25),np.ones(25)]).T
        
    np.testing.assert_equal( h5['electrodes/'+population_name+'/scaling_factors'][:], expectedCoeffs )
    
    h5.close()
    
def test_add_coeffs_backwards(writeNeuron,gids,population_name,data_backwards):
        
    h5File = writeNeuron[0]
    
    h5 = h5py.File(h5File,'r+')
    
    test_data = pd.DataFrame(data=np.arange(25)[np.newaxis,:],columns=data_backwards.columns)
    
    add_data(h5,gids,test_data,population_name)
    
    expectedCoeffs = np.array([np.hstack((np.arange(6,25),np.arange(6))),np.ones(25)]).T
        
    np.testing.assert_equal( h5['electrodes/'+population_name+'/scaling_factors'][:], expectedCoeffs )
    
    h5.close()
    
def test_get_coeffs_lineSource(positions,data_twoSections,electrodePosition,sigma):
    
    columns = data_twoSections.columns
    coeffs = get_coeffs_lineSource(positions,columns,electrodePosition,sigma)
    
    somaDistance = np.sqrt(3*10**2)*1e-6
    expectedSomaCoeff = 1/(4*np.pi*sigma*somaDistance)*1e-9
    
    expectedLineCoeff = get_line_coeffs(np.array([0,0,0]),np.array([0,0,1]),electrodePosition,sigma)
    
    expectedOutput = pd.DataFrame(data=np.hstack((expectedSomaCoeff,expectedLineCoeff))[np.newaxis,:],columns=columns)
    
    pd.testing.assert_frame_equal(coeffs,expectedOutput)
    
def test_lineSource():
    
    segment_positions = [np.array([0,0,0]),np.array([1,0,0])]
    electrodePosition = np.array([2,0,1]) 
    sigma = 1
    
    deltaS = 1 * 1e-6
    h = 1 * 1e-6
    r = 1 * 1e-6
    l = 2 * 1e-6
    
    expectedOutput = 1/(4*np.pi*sigma*deltaS)*np.log( np.abs( (np.sqrt(h**2+r**2) - h)/(np.sqrt(l**2+r**2)-l) ) )
    
    lineCoeff = get_line_coeffs(segment_positions[0],segment_positions[1],electrodePosition,sigma)
    
    np.testing.assert_almost_equal(lineCoeff, expectedOutput*1e-9)

def test_lineSource_2():
    
    segment_positions = [np.array([0,0,0]),np.array([1,0,0])]
    electrodePosition = np.array([-2,0,1]) 
    sigma = 1
    
    deltaS = 1 * 1e-6
    h = -3 * 1e-6
    r = 1 * 1e-6
    l = -2 * 1e-6
    
    expectedOutput = 1/(4*np.pi*sigma*deltaS)*np.log( np.abs( (np.sqrt(h**2+r**2) - h)/(np.sqrt(l**2+r**2)-l) ) )
    
    lineCoeff = get_line_coeffs(segment_positions[0],segment_positions[1],electrodePosition,sigma)
    
    np.testing.assert_almost_equal(lineCoeff, expectedOutput*1e-9)

def test_lineSource_3():
    
    segment_positions = [np.array([0,0,0]),np.array([1,0,0])]
    electrodePosition = np.array([0.5,0,1]) 
    sigma = 1
    
    deltaS = 1 * 1e-6
    h = -0.5 * 1e-6
    r = 1 * 1e-6
    l = 0.5 * 1e-6
    
    expectedOutput = 1/(4*np.pi*sigma*deltaS)*np.log( np.abs( (np.sqrt(h**2+r**2) - h)/(np.sqrt(l**2+r**2)-l) ) )
    
    lineCoeff = get_line_coeffs(segment_positions[0],segment_positions[1],electrodePosition,sigma)
    
    np.testing.assert_almost_equal(lineCoeff, expectedOutput*1e-9)
    
def test_get_coeffs_pointSource(positions,electrodePosition,sigma,gids):
    
    newPositions = getSegmentMidpts(positions,gids)
    coeffs = get_coeffs_pointSource(newPositions,electrodePosition,sigma)
    
    somaDistance = np.sqrt(3*10**2)*1e-6
    expectedSomaCoeff = 1/(4*np.pi*sigma*somaDistance)*1e-9
    
    segmentDistance = np.sqrt(10**2+10**2+(10-.5)**2)*1e-6
    
    expectedSegmentCoeff = 1/(4*np.pi*sigma*segmentDistance)*1e-9
    
    expectedOutput = pd.DataFrame(data=np.hstack((expectedSomaCoeff,expectedSegmentCoeff))[np.newaxis,:],columns=newPositions.columns)
    
    pd.testing.assert_frame_equal(coeffs,expectedOutput)
    
def test_get_coeffs_reciprocity(positions,write_potentialField,gids):
    
    testPositions = getSegmentMidpts(positions,gids)
    potentials = get_coeffs_reciprocity(testPositions,write_potentialField)
    
    columns = [[1,1],[0,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])
    
    expectedPotential = pd.DataFrame(data=np.array([0,0.5e-6])[np.newaxis,:],columns=columnMultiIndex)
    
    pd.testing.assert_frame_equal(potentials,expectedPotential)
    
def test_get_coeffs_dipoleReciprocity(positions,write_EField,gids):
    
    testPositions = getSegmentMidpts(positions,gids)
    
    center = testPositions.mean(axis=1)
    
    potentials = get_coeffs_dipoleReciprocity(testPositions,write_EField,center)
        
    columns = [[1,1],[0,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])
    
    expectedPotential = pd.DataFrame(data=-1*np.array([0.5e-6,0])[np.newaxis,:]**2,columns=columnMultiIndex)
        
    pd.testing.assert_frame_equal(potentials,expectedPotential)
    
def test_sort_electrode_names():
    
    electrodes = [0,1,10,'S1nonbarrel_neurons',3]
    population = 'S1nonbarrel_neurons'
    
    electrodeList = sort_electrode_names(electrodes,population)
    
    assert np.array_equal(electrodeList,[0,1,3,10])

def test_electrodeType():
    
    electrodeTypes = ['PointSource','LineSource','LineSourceOld','Reciprocity','DipoleReciprocity']
    
    for electrodeType in electrodeTypes:
        assert ElectrodeType(electrodeType) == 0
    
    badType = 'sadasd'
    
    with pytest.raises(AssertionError):
        ElectrodeType(badType)
    
    
