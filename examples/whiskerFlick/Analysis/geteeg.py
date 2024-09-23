import bluepysnap as bp
from bluepysnap.sonata_constants import Node
import numpy as np
import pandas as pd
import sys
from mpi4py import MPI
import os 

rank = MPI.COMM_WORLD.Get_rank()

nranks = MPI.COMM_WORLD.Get_size()

numfolders = 10

folder = '../whiskerFlickSim/original/'+str(rank % numfolders)


ranksPerFolder = int(nranks/numfolders)

typesPerRank = int(60/ranksPerFolder)

s = bp.Simulation(folder+'/simulation_config.json')
population_name = s.reports['lfp_report'].population_names[0]

r  = s.reports['lfp_report'][population_name]

types = np.sort(list(s.circuit.nodes.property_values('mtype')))

tIdx = int(rank/numfolders)

for t in np.arange(tIdx*typesPerRank,(tIdx+1)*typesPerRank):

    eeg = r.get(group=types[t],t_start=1975,t_stop=2200)
    eeg.columns = pd.MultiIndex.from_product((np.unique(eeg.columns.get_level_values(0)),['Forelimb_EEG_Reciprocity','Testing']),names=['gid','electrode']) #Adds names for the column indice
    eeg = eeg.sum(axis=1,level='electrode')
    
    eeg.to_pickle(folder+'/pkls/eeg'+types[t]+'.pkl')

    if folder == '../whiskerFlickSim/original/0':
        print(eeg)
