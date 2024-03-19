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


source ~/bluerecording-dev/bin/activate
spack env activate bluerecording-dev

export NEURODAMUS_PYTHON=../../../../neurodamus


srun dplace ../../../../x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
