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

spack env activate neurodamus
module load unstable
module load neurodamus-neocortex/unstable neuron/develop py-neurodamus/unstable

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json
