#!/bin/bash -l
#SBATCH --account=proj85
#SBATCH --partition=prod
#SBATCH --nodes=32
#SBATCH --cpus-per-task=2
#SBATCH --mem=0
#SBATCH --constraint=cpu
#SBATCH --exclusive
#SBATCH --time=24:00:00
#SBATCH --job-name=CortexNrdmsPySim

# SPDX-License-Identifier: GPL-3.0-or-later

#module load unstable
#module load neurodamus-thalamus py-neurodamus

module load archive/2024-05 neurodamus-thalamus
export PATH=/gpfs/bbp.cscs.ch/project/proj83/home/weji/combined_models_new_2024_05/x86_64:$PATH
export PYTHONPATH=/gpfs/bbp.cscs.ch/project/proj83/home/weji/neurodamus-3.3.0-gj-correction:$PYTHONPATH

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
