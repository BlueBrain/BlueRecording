#!/bin/bash -l
#SBATCH --account=proj85
#SBATCH --partition=prod
#SBATCH --nodes=40
#SBATCH --cpus-per-task=2
#SBATCH --mem=0
#SBATCH --constraint=cpu
#SBATCH --exclusive
#SBATCH --time=24:00:00
#SBATCH --job-name=CortexNrdmsPySim

# SPDX-License-Identifier: GPL-3.0-or-later


spack env activate hippocampus
module load neurodamus-hippocampus/develop 

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config_big.json --lb-mode=RoundRobin
