# SPDX-License-Identifier: GPL-3.0-or-later
import pytest
import numpy as np
from bluerecording.utils import *

def test_conductance_to_floats():

    conductanceList = ['0.1','333','23.5']
    pathList = ['assasd','k23hbe2k','sndkjansd']
    badList = ['222.0','asdsd']

    isConductance, inputList = conductance_to_floats(conductanceList)

    assert isConductance == 1
    assert inputList == [0.1,333.0,23.5]

    isConductance, inputList = conductance_to_floats(pathList)

    assert isConductance == 0
    assert inputList == pathList

    with pytest.raises(AssertionError) as excinfo:
        conductance_to_floats(badList)

    assert str(excinfo.value) == 'Mix of numbers and strings in input'

def test_splitInput():

    inputString = '1 2 3'
    isconductance, output = splitInput(inputString)
    assert output == [1.,2.,3.]
    assert isconductance == 1

    inputString = 'a b c'
    isconductance, output = splitInput(inputString)
    assert output == ['a','b','c']
    assert isconductance == 0

    inputString = '1'
    isconductance, output = splitInput(inputString)
    assert isconductance == 1
    assert output == [1]

def test_process_writeH5_inputs():

    path_to_simconfig = 'path_to_simconfig'
    segment_position_folder = 'segment_position_folder'
    outputfile = 'outputfile'
    neurons_per_file = 1000
    files_per_folder = 50
    
    sysArgVBase = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder]

    sigma, path_to_h5 = process_writeH5_inputs(sysArgVBase)

    assert sigma == [0.277]
    assert path_to_h5 is None

    sigmaString = '0.7'
    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigmaString] 
    sigma, path_to_h5 = process_writeH5_inputs(sysArgV)
    assert sigma == [0.7]
    assert path_to_h5 is None

    path = 'aaa.h5'
    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,path]
    sigma, path_to_h5 = process_writeH5_inputs(sysArgV)
    assert sigma == [0.277]
    assert path_to_h5 == ['aaa.h5']

    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigmaString,path] 
    sigma, path_to_h5 = process_writeH5_inputs(sysArgV)
    assert sigma == [0.7]
    assert path_to_h5 == ['aaa.h5']

def test_getSimulationInfo(path_to_simconfig_with_output):
    
    report, nodeIds = getSimulationInfo(path_to_simconfig_with_output)
            
    assert nodeIds == 0

def test_getPopulationName(path_to_simconfig_with_output):

    assert getPopulationName(path_to_simconfig_with_output)=='S1nonbarrel_neurons'

def test_getCircuitPath(path_to_simconfig_with_output,expected_circuit_path):

    assert getCircuitPath(path_to_simconfig_with_output)==expected_circuit_path

def test_atlasInfo(path_to_simconfig_with_atlas):
    
    electrodesOut = np.array([[0,0,0],[0,0,1]])
    
    regionList, layerList = getAtlasInfo(path_to_simconfig_with_atlas,electrodesOut)
    
    assert regionList == ['Outside','Outside']
    assert layerList == ['Outside','Outside']
    
    electrode_in = np.array([[3251.37,-777,-2178.89]])
    
    regionList, layerList = getAtlasInfo(path_to_simconfig_with_atlas,electrode_in)
    
    assert regionList == ['S1FL']
    assert layerList == ['L5']

