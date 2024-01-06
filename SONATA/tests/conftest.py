import pytest 
import pandas as pd
import bluepysnap as bp
import numpy as np
from morphio import PointLevel, SectionType
from morphio import Morphology
import h5py
from writeH5_prelim import *

@pytest.fixture(scope='session')
def path_to_weights_file(tmpdir_factory):
    fn = tmpdir_factory.mktemp('data').join('testfile.h5')
    return fn

@pytest.fixture(scope='session')
def path_to_morphology_file(tmpdir_factory):
    fn = tmpdir_factory.mktemp('data').join('morph.h5')
    return fn

@pytest.fixture(scope='session')
def path_to_potentialfield_file(tmpdir_factory):
    fn = tmpdir_factory.mktemp('data').join('potential.h5')
    return fn

@pytest.fixture(scope="module")
def data():

    '''
    Defines a data frame mimicking a voltage report, with columns containing gids and section ids
    '''
    
    columns = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2],[0,1,1,1,1,1,2,2,2,2,2,3,3,3,10,10,10,10,10,0,1,1,1,1,1]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])

    data = pd.DataFrame(data=np.zeros([1,len(columns[0])]),columns=columnMultiIndex)
    
    return data

@pytest.fixture(scope="module")
def data_backwards():

    '''
    Defines a data frame mimicking a voltage report, with columns containing gids and section ids
    '''
    
    columns = [[2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],[0,1,1,1,1,1,0,1,1,1,1,1,2,2,2,2,2,3,3,3,10,10,10,10,10]]
    
    columnIdx = list(zip(*columns))
    
    columnMultiIndex = pd.MultiIndex.from_tuples(columnIdx,names=['id','section'])

    data = pd.DataFrame(data=np.zeros([1,len(columns[0])]),columns=columnMultiIndex)
    
    return data

@pytest.fixture(scope="module")
def secCounts(data):

    '''
    Defines a data frame containing gids and section ids, equivalent to the column indices from a voltage report
    '''

    sectionIds = data.columns.to_frame() # Data frame containing gids and section ids
    sectionIds.index = range(len(sectionIds))
    
    return sectionIds

@pytest.fixture(scope="module")
def electrodes():
    
    electrodes = {'name':{'position':np.array([1,2,3]),'type':'Reciprocity','region':'Outside','layer':'Outside'}}
    
    return electrodes

@pytest.fixture(scope="module")
def gids():
    
    return [1,2]

@pytest.fixture(scope="module")
def population_name():
    
    return 'testPopulation'

@pytest.fixture(scope="module")
def somaPos():
    
    return np.array([0,0,0])

@pytest.fixture(scope="module")
def write_ElectrodeFileStructure(path_to_weights_file, electrodes, gids, population_name):
    
    '''
    Creates h5 file, without any weights
    '''
    

    h5file = h5py.File(path_to_weights_file,'w')

    h5 = ElectrodeFileStructure(h5file, gids, electrodes, population_name, circuit='test') # Initializes fields in h5 file

    h5file.close()
    
    return path_to_weights_file, h5

@pytest.fixture(scope="module")
def writeNeuron(write_ElectrodeFileStructure, secCounts, electrodes, population_name):
    
    path, h5 = write_ElectrodeFileStructure
        
    h5file = h5py.File(path,'r+')
        
    write_all_neuron(secCounts, population_name, h5, h5file, electrodes)
    
    h5file.close()
    
    return path, h5

@pytest.fixture(scope="module")
def morphology_trivial(path_to_morphology_file):

    file = h5py.File(path_to_morphology_file,'w')

    structureData = np.array([[0,1,-1],[1,2,0]]) # One soma with 1 3d point, 1 axon sections with 2 points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[1,0,0,.3]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(path_to_morphology_file)

@pytest.fixture(scope="module")
def morphology_short(path_to_morphology_file):

    file = h5py.File(path_to_morphology_file,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[5,2,1],[7,3,0]]) # One soma with 1 3d point, 2 axon sections with4 and 3 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1,.3],[0,0,2,.3],[0,0,3,1],[0,0,3,1],[0,0,4,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(path_to_morphology_file)

@pytest.fixture(scope="module")
def morphology(path_to_morphology_file):


    file = h5py.File(path_to_morphology_file,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[5,2,1],[7,3,0]]) # One soma with 1 3d point, 2 axon sections with 4 and 2 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1,.3],[0,0,2,.3],[0,0,3,1],[0,0,3,1],[0,0,1073,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(path_to_morphology_file)

@pytest.fixture(scope="module")
def morphology_farAxon(path_to_morphology_file):


    file = h5py.File(path_to_morphology_file,'w')

    structureData = np.array([[0,1,-1],[1,2,0],[3,3,0]]) # One soma with 1 3d point, 1 axon sections with 2 points, one basal dendrite with undefined number of points

    structure = file.create_dataset('structure',data=structureData)

    pointsData = np.array([[0,0,0,1],[0,0,0,1],[0,0,1073,1],[0,0,0,1],[10,0,0,5],[100,0,0,5]])
    points = file.create_dataset('points',data=pointsData)

    file.close()

    return Morphology(path_to_morphology_file)

@pytest.fixture(scope="module")
def write_potentialField(path_to_potentialfield_file):


    file = h5py.File(path_to_potentialfield_file,'w')

    file.create_dataset('CurrentApplied',data=1)

    xaxis = np.linspace(-10,10)*1e-6
    yaxis = np.linspace(-10,10)*1e-6
    zaxis = np.linspace(-10,10)*1e-6
    realImag = np.array([0,1])

    meshes = file.create_group('Meshes')
    firstdatafield = meshes.create_group('FirstDataField')
    axis_x = firstdatafield.create_dataset('axis_x',data=xaxis)
    axis_y = firstdatafield.create_dataset('axis_y',data=yaxis)
    axis_z = firstdatafield.create_dataset('axis_z',data=zaxis)
    
    fieldgroups = file.create_group('FieldGroups')
    randomname = fieldgroups.create_group('randomname')
    allfields = randomname.create_group('AllFields')
    potential = allfields.create_group('EM Potential(x,y,z,f0)')
    obj = potential.create_group('_Object')
    snapshot = obj.create_group('Snapshots')
    field0 = snapshot.create_group('0')
    
    xd, yd, zd,rd = np.meshgrid(xaxis,yaxis,zaxis,realImag,indexing='ij')
    comp0 = field0.create_dataset('comp0',data =zd)
    

    file.close()

    return path_to_potentialfield_file

@pytest.fixture(scope="module")
def write_EField(path_to_potentialfield_file):


    file = h5py.File(path_to_potentialfield_file,'w')

    file.create_dataset('CurrentApplied',data=1)
    
    xaxis = np.linspace(-10,10)*1e-6
    yaxis = np.linspace(-10,10)*1e-6
    zaxis = np.linspace(-10,10)*1e-6

    xcenter = (xaxis[1:]+xaxis[:-1])/2
    ycenter = (yaxis[1:]+yaxis[:-1])/2
    zcenter = (zaxis[1:]+zaxis[:-1])/2

    realImag = np.array([0,1])

    meshes = file.create_group('Meshes')
    firstdatafield = meshes.create_group('FirstDataField')
    axis_x = firstdatafield.create_dataset('axis_x',data=xaxis)
    axis_y = firstdatafield.create_dataset('axis_y',data=yaxis)
    axis_z = firstdatafield.create_dataset('axis_z',data=zaxis)
    
    fieldgroups = file.create_group('FieldGroups')
    randomname = fieldgroups.create_group('randomname')
    allfields = randomname.create_group('AllFields')
    potential = allfields.create_group('EM E(x,y,z,f0)')
    obj = potential.create_group('_Object')
    snapshot = obj.create_group('Snapshots')
    field0 = snapshot.create_group('0')
    
    xd, __, __, __ = np.meshgrid(xcenter,yaxis,zaxis,realImag,indexing='ij')
    __, yd, __, __ = np.meshgrid(xaxis,ycenter,zaxis,realImag,indexing='ij')
    __, __, zd, __ = np.meshgrid(xaxis,yaxis,zcenter,realImag,indexing='ij')
    
    comp0 = field0.create_dataset('comp0',data =xd)
    comp1 = field0.create_dataset('comp1',data =yd)
    comp2 = field0.create_dataset('comp2',data =zd)
    

    file.close()

    return path_to_potentialfield_file
