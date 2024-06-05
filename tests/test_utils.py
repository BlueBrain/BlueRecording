# SPDX-License-Identifier: GPL-3.0-or-later
import pytest
import numpy as np
from bluerecording.utils import *

def test_process_inputList():

    conductanceList = ['0.1','333','23.5']
    pathList = ['assasd','k23hbe2k','sndkjansd']
    badList = ['222.0','asdsd']
    badSubsampling = ['subsampling','5']
    radiusList = ['radius','200']

    inputType, inputList = process_inputList(conductanceList)

    assert inputType == 'conductance'
    assert inputList == [0.1,333.0,23.5]

    inputType, inputList = process_inputList(pathList)

    assert inputType == 'paths'
    assert inputList == pathList

    with pytest.raises(AssertionError) as excinfo:
        process_inputList(badList)

    assert str(excinfo.value) == 'Mix of numbers and strings in input'

    with pytest.raises(AssertionError) as excinfo:
        process_inputList(badSubsampling)

    assert str(excinfo.value) == 'Subsampling should be given in the form of a range, like \'5:9\', not as an integer'

    inputType, inputList = process_inputList(radiusList)

    assert inputType == 'radius'
    assert inputList == [200.]

def test_splitInput():

    inputString = '1 2 3'
    inputType, output = splitInput(inputString)
    assert output == [1.,2.,3.]
    assert inputType == 'conductance'

    inputString = 'a b c'
    inputType, output = splitInput(inputString)
    assert output == ['a','b','c']
    assert inputType == 'paths'

    inputString = '1'
    inputType, output = splitInput(inputString)
    assert inputType == 'conductance'
    assert output == [1]

    inputString = 'radius 1'
    inputType, output = splitInput(inputString)
    assert inputType == 'radius'
    assert output == [1]

    inputString = 'subsampling 5:10'
    inputType, output = splitInput(inputString)
    assert inputType == 'subsampling'
    assert output == ['5:10']

def test_processSubsampling():

    inputString = '5:10'
    output = processSubsampling(inputString)
    assert output == [5, 10]

def test_process_writeH5_inputs():

    path_to_simconfig = 'path_to_simconfig'
    segment_position_folder = 'segment_position_folder'
    outputfile = 'outputfile'
    neurons_per_file = 1000
    files_per_folder = 50
    
    sysArgVBase = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder]

    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgVBase)

    assert sigma == [0.277]
    assert path_to_h5 is None
    assert radius == [50]
    assert subsampling is None

    sigmaString = '0.7'
    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigmaString] 
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.7]
    assert path_to_h5 is None
    assert radius == [50]
    assert subsampling is None

    path = 'aaa.h5'
    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,path]
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.277]
    assert path_to_h5 == ['aaa.h5']
    assert radius == [50]
    assert subsampling is None

    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigmaString,path] 
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.7]
    assert path_to_h5 == ['aaa.h5']
    assert radius == [50]
    assert subsampling is None
   
    radiusString = 'radius 10'
    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,radiusString] 
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.277]
    assert path_to_h5 is None
    assert radius == [10.]
    assert subsampling is None

    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,sigmaString,radiusString] 
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.7]
    assert path_to_h5 is None
    assert radius == [10.]
    assert subsampling is None

    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,radiusString,path,sigmaString] 
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.7]
    assert path_to_h5 == ['aaa.h5']
    assert radius == [10.]
    assert subsampling is None

    samplingString = 'subsampling 5:9 9:14'
    sysArgV = ['functionName',path_to_simconfig,segment_position_folder,outputfile,neurons_per_file,files_per_folder,path,samplingString] 
    sigma, path_to_h5, radius, subsampling = process_writeH5_inputs(sysArgV)
    assert sigma == [0.277]
    assert path_to_h5 == ['aaa.h5']
    assert radius == [50.]
    assert subsampling == ['5:9','9:14']

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

