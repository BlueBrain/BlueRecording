# SPDX-License-Identifier: GPL-3.0-or-later
import pytest
import numpy as np
from bluerecording.utils import *

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