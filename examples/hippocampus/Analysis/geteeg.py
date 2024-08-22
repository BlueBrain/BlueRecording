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

tIdx = MPI.COMM_WORLD.Get_rank()

nranks = MPI.COMM_WORLD.Get_size()


s = bp.Simulation('../hippocampusSim/simulation_config_small.json')
population_name = s.reports['lfp_report'].population_names[0]

r  = s.reports['lfp_report'][population_name]

types = np.sort(list(s.circuit.nodes.property_values('mtype')))

nodes = s.circuit.nodes

typeIds = nodes.ids(group=types[tIdx]).get_ids()
numIter = int(np.ceil(len(typeIds)/10000))

for iteration in range(numIter):
    
    try:
        eeg = r.get(group=typeIds[iteration*10000:(iteration+1)*10000],t_step=1) 
    except:
        eeg = r.get(group=typeIds[iteration*10000:],t_step=1)
    
    eeg.columns = pd.MultiIndex.from_product((np.unique(eeg.columns.get_level_values(0)),np.unique(eeg.columns.get_level_values(1))),names=['gid','electrode']) #Adds names for the column indice
    eeg = eeg.T.groupby(level='electrode').sum().T # Sums over all gids
    
    
    eeg.to_pickle('../hippocampusSim/pkls/eeg_'+types[tIdx]+'_'+str(iteration)+'.pkl')
