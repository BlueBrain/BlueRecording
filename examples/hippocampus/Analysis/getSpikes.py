# SPDX-License-Identifier: GPL-3.0-or-later
import bluepysnap as bp
from bluepysnap.sonata_constants import Node
import numpy as np
import pandas as pd
import sys
from mpi4py import MPI
import os 

'''
Iterates through subregions and cell types, and saves the spike reports for each subregion-cell type combination to a pickle file
'''

rank = MPI.COMM_WORLD.Get_rank()

nranks = MPI.COMM_WORLD.Get_size()

s = bp.Simulation('../hippocampusSim/simulation_config_small.json')
population_name = s.reports['lfp_report'].population_names[0]

types = np.sort(list(s.circuit.nodes.property_values('mtype')))

tIdx = int(rank)

nodes = s.circuit.nodes
typeIds = nodes.ids(group=types[tIdx])

spikeReports = s.spikes[population_name]

spikes = spikeReports.get(group=typeIds,t_start=9000) # Ignore first 2 seconds to exclude transient


spikes.to_pickle('../hippocampusSim/pkls/spikes_'+types[tIdx]+'.pkl')
