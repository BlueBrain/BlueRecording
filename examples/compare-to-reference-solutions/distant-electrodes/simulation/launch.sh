#!/bin/bash -l
#SBATCH --account=proj83
#SBATCH --partition=prod_small
#SBATCH --nodes=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=0
#SBATCH --constraint=cpu
#SBATCH --exclusive
#SBATCH --time=2:00:00
#SBATCH --job-name=CortexNrdmsPySim

module load unstable neurodamus-neocortex

export NEURODAMUS_PYTHON=/gpfs/bbp.cscs.ch/project/proj83/tharayil/neurodamus

srun -n1 ./x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json
