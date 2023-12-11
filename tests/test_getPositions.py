import pytest
import pandas as pd
import bluepy as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from getPositions import *

@pytest.fixture
def h5File(tmp_path):

    filename = tmp_path / 'testfile.h5'

    file = h5py.File(filename,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[5,2,1],[7,3,0]]) # One soma with 1 3d point, 2 axon sections with 4 and 2 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1,.3],[0,0,2,.3],[0,0,3,1],[0,0,3,1],[0,0,1073,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(filename)

@pytest.fixture
def h5File_farAxon(tmp_path):

    filename = tmp_path / 'testfile.h5'

    file = h5py.File(filename,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[3,3,0]]) # One soma with 1 3d point, 1 axon sections with 2 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1073,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(filename)

@pytest.fixture
def h5File_short(tmp_path):

    filename = tmp_path / 'testfile.h5'

    file = h5py.File(filename,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[5,2,1],[7,3,0]]) # One soma with 1 3d point, 2 axon sections with 4 and 2 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1,.3],[0,0,2,.3],[0,0,3,1],[0,0,3,1],[0,0,4,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(filename)


@pytest.fixture
def data():

    '''
    Defines a data frame mimicking a voltage report, with columns containing gids and section ids
    '''
    
    columns = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],[0,1,1,1,1,1,2,2,2,2,2,3,3,3,10,10,10,10,10]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['gid',bp.Section.ID])

    sec_counts = pd.DataFrame(data=np.zeros([1,len(columns[0])]),columns=columnMultiIndex)
    
    return sec_counts

def test_getSomaPosition(h5File, data):

    
    i = 1
    
    expectedSomaPos = np.array([[0,0,0]])
    
    somaPos = getSomaPosition(i,h5File,data)
    
    np.testing.assert_equal(somaPos,expectedSomaPos)

def test_get_axon_points(h5File):
    
    points, lengths = get_axon_points(h5File)
    expectedLengths = np.array([0,1,2,3,1073])
   
    expectedPoints = np.array([[0,0,0],[0,0,1],[0,0,2],[0,0,3],[0,0,1073]])
    np.testing.assert_almost_equal(lengths,expectedLengths,decimal=2)
    np.testing.assert_almost_equal(points,expectedPoints,decimal=2)
    
def test_get_axon_points_extrapolate(h5File_short):
    
    points, lengths = get_axon_points(h5File_short)
    expectedLengths = np.array([0,1,2,3,4,1060])
   
    expectedPoints = np.array([[0,0,0],[0,0,1],[0,0,2],[0,0,3],[0,0,4],[0,0,1060]])
    np.testing.assert_almost_equal(lengths,expectedLengths,decimal=2)
    
    np.testing.assert_almost_equal(points,expectedPoints,decimal=2)
    
def test_getNewIdx(data):
    
    colIdx = data.columns 
    expectedColumns = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],[0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,10,10,10,10,10,10]]
    
    expectedIdx = list(zip(*expectedColumns))
    
    expectedMultiIndex = pd.MultiIndex.from_tuples(expectedIdx,names=['gid',bp.Section.ID])
    
    newIdx = getNewIndex(colIdx)
    
    pd.testing.assert_index_equal(newIdx, expectedMultiIndex)
    
def test_interpolate_dendrite(data, h5File):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[3]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    sec = h5File.sections[secName-1] # If the section is not dendritic, this will trhow an Exception
    pts = sec.points

    secPts = np.array(pts)

    segPos = interp_points(secPts,numCompartments)
    
    expectedSegPos = np.array([[0,0,0],[33.33,0,0],[66.66,0,0],[100,0,0]])

    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS(data,h5File):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,0],[0,0,6],[0,0,12],[0,0,18],[0,0,24],[0,0,30]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_farAxon(data,h5File):
    
    '''
    Makes sure that edge case in which only the soma itself is less than 30 um away from the soma is properly handled
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,0],[0,0,6],[0,0,12],[0,0,18],[0,0,24],[0,0,30]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_short(data,h5File_short):
    
    '''
    Tests the case where no point is greater than 30 um away from the soma
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File_short)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,0],[0,0,6],[0,0,12],[0,0,18],[0,0,24],[0,0,30]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_2(data,h5File):
    
    '''
    Tests the case where no point is between 30 and 60 um from the soma, but there is one farther than 60 um
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[2]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,30],[0,0,36],[0,0,42],[0,0,48],[0,0,54],[0,0,60]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_2_short(data,h5File_short):
    
    '''
    Tests the case where no points greater than 30 um from the soma
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[2]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File_short)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,30],[0,0,36],[0,0,42],[0,0,48],[0,0,54],[0,0,60]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_myelin(data,h5File):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[-1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,60],[0,0,260],[0,0,460],[0,0,660],[0,0,860],[0,0,1060]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_myelin_short(data,h5File_short):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[-1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(h5File_short)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,60],[0,0,260],[0,0,460],[0,0,660],[0,0,860],[0,0,1060]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
