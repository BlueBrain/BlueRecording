# SPDX-License-Identifier: GPL-3.0-or-later
import bluepysnap as bp
from bluepysnap.sonata_constants import Node
import numpy as np
import pandas as pd
import sys
from mpi4py import MPI
import os 

'''
Iterates through subregions and cell types, and saves the extracellular recordings summed over all cells in each subregion-cell type combination to a pickle file
'''

rank = MPI.COMM_WORLD.Get_rank()

nranks = MPI.COMM_WORLD.Get_size()


s = bp.Simulation('../thalamusSimulation/simulation_config.json')
population_name = s.reports['lfp_report'].population_names[0]

r  = s.reports['lfp_report'][population_name]

types = np.sort(list(s.circuit.nodes.property_values('mtype')))

nodes = s.circuit.nodes

for tIdx in range(len(types)):
    typeIds = nodes.ids(group=types[tIdx])
    
    
    eeg = r.get(group=typeIds,t_start=2000,t_step=1) # Ignore first 2 seconds to exclude transient
    
    
    eeg.columns = pd.MultiIndex.from_product((np.unique(eeg.columns.get_level_values(0)),['Electrode','Testing']),names=['gid','electrode']) #Adds names for the column indices
    eeg = eeg.T.groupby(level='electrode').sum().T # Sums over all gids
    
    
    eeg.to_pickle('../thalamusSimulation/pkls/eeg_'+types[tIdx]+'.pkl')
