import pytest
import pandas as pd
import bluepy as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from getPositions import getSomaPosition
from getPositions import get_axon_points

@pytest.fixture
def h5File(tmp_path):

    filename = tmp_path / 'testfile.h5'

    file = h5py.File(filename,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[5,2,1],[7,3,0]]) # One soma with 1 3d point, 2 axon sections with 4 and 2 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1,.3],[0,0,2,.3],[0,0,3,1],[0,0,3,1],[0,1070,3,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
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
    
    columns = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1],[0,1,1,1,1,1,2,2,2,2,2,3,3,3]]
    
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
    expectedLengths = np.array([0,1,2,3,3,1073])
   
    expectedPoints = np.array([[0,0,0],[0,0,1],[0,0,2],[0,0,3],[0,0,3],[0,1070,3]])
    np.testing.assert_almost_equal(lengths,expectedLengths,decimal=2)
    np.testing.assert_almost_equal(points,expectedPoints,decimal=2)
    
def test_get_axon_points_extrapolate(h5File_short):
    
    points, lengths = get_axon_points(h5File_short)
    expectedLengths = np.array([0,1,2,3,3,4,1060])
   
    expectedPoints = np.array([[0,0,0],[0,0,1],[0,0,2],[0,0,3],[0,0,3],[0,0,4],[0,0,1056]])
    np.testing.assert_almost_equal(lengths,expectedLengths,decimal=2)
    
    print(points)
    np.testing.assert_almost_equal(points,expectedPoints,decimal=2)
    
