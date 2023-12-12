import pytest
import pandas as pd
import bluepy as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from getPositions import *


def test_getSomaPosition(morphology, data):

    
    i = 1
    
    expectedSomaPos = np.array([[0,0,0]])
    
    somaPos = getSomaPosition(i,morphology,data)
    
    np.testing.assert_equal(somaPos,expectedSomaPos)

def test_get_axon_points(morphology):
    
    points, lengths = get_axon_points(morphology)
    expectedLengths = np.array([0,1,2,3,1073])
   
    expectedPoints = np.array([[0,0,0],[0,0,1],[0,0,2],[0,0,3],[0,0,1073]])
    np.testing.assert_almost_equal(lengths,expectedLengths,decimal=2)
    np.testing.assert_almost_equal(points,expectedPoints,decimal=2)
    
def test_get_axon_points_extrapolate(morphology_short):
    
    points, lengths = get_axon_points(morphology_short)
    expectedLengths = np.array([0,1,2,3,4,1060])
   
    expectedPoints = np.array([[0,0,0],[0,0,1],[0,0,2],[0,0,3],[0,0,4],[0,0,1060]])
    np.testing.assert_almost_equal(lengths,expectedLengths,decimal=2)
    
    np.testing.assert_almost_equal(points,expectedPoints,decimal=2)
    
def test_getNewIdx(data):
    
    colIdx = data.columns 
    expectedColumns = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2],[0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,10,10,10,10,10,10,0,1,1,1,1,1,1]]
    
    expectedIdx = list(zip(*expectedColumns))
    
    expectedMultiIndex = pd.MultiIndex.from_tuples(expectedIdx,names=['gid',bp.Section.ID])
    
    newIdx = getNewIndex(colIdx)
    
    pd.testing.assert_index_equal(newIdx, expectedMultiIndex)
    
def test_interpolate_dendrite(data, morphology):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[3]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    sec = morphology.sections[secName-1] # If the section is not dendritic, this will trhow an Exception
    pts = sec.points

    secPts = np.array(pts)

    segPos = interp_points(secPts,numCompartments)
    
    expectedSegPos = np.array([[0,0,0],[33.33,0,0],[66.66,0,0],[100,0,0]])

    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS(data,morphology):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,0],[0,0,6],[0,0,12],[0,0,18],[0,0,24],[0,0,30]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_farAxon(data,morphology_farAxon):
    
    '''
    Makes sure that edge case in which only the soma itself is less than 30 um away from the soma is properly handled
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology_farAxon)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,0],[0,0,6],[0,0,12],[0,0,18],[0,0,24],[0,0,30]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_short(data,morphology_short):
    
    '''
    Tests the case where no point is greater than 30 um away from the soma
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology_short)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,0],[0,0,6],[0,0,12],[0,0,18],[0,0,24],[0,0,30]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_2(data,morphology):
    
    '''
    Tests the case where no point is between 30 and 60 um from the soma, but there is one farther than 60 um
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[2]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,30],[0,0,36],[0,0,42],[0,0,48],[0,0,54],[0,0,60]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_AIS_2_short(data,morphology_short):
    
    '''
    Tests the case where no points greater than 30 um from the soma
    '''
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[2]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology_short)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,30],[0,0,36],[0,0,42],[0,0,48],[0,0,54],[0,0,60]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_myelin(data,morphology):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[-1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,60],[0,0,260],[0,0,460],[0,0,660],[0,0,860],[0,0,1060]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
    
def test_interpolate_myelin_short(data,morphology_short):
    
    colIdx = data.columns # GID and Section IDs for each cell
    cols = np.array(list(data.columns))
    
    i = 1 # gid
    
    sections = np.unique(cols[np.where(cols[:,0]==i),1:].flatten()) # List of sections for the given neuron
    
    secName = sections[-1]
    
    numCompartments = np.shape(data[i][secName])[-1]
    
    axonPoints, runningLens = get_axon_points(morphology_short)
    
    somaPos = np.array([0,0,0])
    
    segPos = interp_points_axon(axonPoints,runningLens,secName,numCompartments,somaPos)
    
    expectedSegPos = np.array([[0,0,60],[0,0,260],[0,0,460],[0,0,660],[0,0,860],[0,0,1060]])
    
    np.testing.assert_almost_equal(segPos,expectedSegPos,decimal=2)
