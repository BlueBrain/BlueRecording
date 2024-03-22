#!/bin/bash -l
#SBATCH --account=proj83
#SBATCH --partition=prod
#SBATCH --nodes=400
#SBATCH --cpus-per-task=2
#SBATCH --mem=0
#SBATCH --constraint=cpu
#SBATCH --exclusive
#SBATCH --time=24:00:00
#SBATCH --job-name=CortexNrdmsPySim

spack env activate bluerecording-dev
source ~/bluerecording-env/bin/activate

export NEURODAMUS_PYTHON=../../../../neurodamus


srun dplace ../../../../x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
