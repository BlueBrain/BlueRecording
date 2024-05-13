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


s = bp.Simulation('../sscxSimulation/simulation_config.json')
population_name = s.reports['lfp_report'].population_names[0]

r  = s.reports['lfp_report'][population_name]

types = np.sort(list(s.circuit.nodes.property_values('mtype')))
regions = ['S1FL','S1DZ','S1DZO','S1HL','S1Sh','S1Tr','S1ULp']

tIdx = int(rank/len(regions))
rIdx = rank % len(regions)

nodes = s.circuit.nodes
regionIds = nodes.ids(group=regions[rIdx])
typeIds = nodes.ids(group=types[tIdx])

ids = regionIds.intersection(typeIds).get_ids()

eeg = r.get(group=ids,t_start=2000,t_step=1) # Ignore first 2 seconds to exclude transient


eeg.columns = pd.MultiIndex.from_product((np.unique(eeg.columns.get_level_values(0)),['Forelimb_ECoG_Reciprocity','Forelimb_EEG_Reciprocity','Forelimb_LFP_Reciprocity','Forelimb_ECoG_Dipole','Forelimb_EEG_Dipole','Forelimb_LFP_Dipole','Forelimb_ECoG_LineSource','Forelimb_EEG_LineSource','Foreelimb_LFP_LineSource','Reference_LineSource','Forelimb_ECoG_PointSource','Forelimb_EEG_PointSource','Forelimb_LFP_PointSource','Reference_PointSource','Testing']),names=['gid','electrode']) #Adds names for the column indices
eeg = eeg.T.groupby(level='electrode').sum().T # Sums over all gids


eeg.to_pickle('../sscxSimulation/pkls/eeg_'+regions[rIdx]+'_'+types[tIdx]+'.pkl')
