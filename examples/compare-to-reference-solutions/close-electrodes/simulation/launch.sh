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


spack env activate bluerecording
module load unstable
module load neurodamus-neocortex/42.42.42_conductance neuron/9.0.a16_conductance 
module load py-neurodamus/3.2.conductance 

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
