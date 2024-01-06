#!/bin/bash -l
#SBATCH --account=proj83
#SBATCH --partition=prod
#SBATCH --nodes=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=0
#SBATCH --constraint=cpu
#SBATCH --exclusive
#SBATCH --time=24:00:00
#SBATCH --job-name=CortexNrdmsPySim

module load archive/2023-11
module load neurodamus-neocortex/1.13-2.16.6-2.8.1 py-neurodamus/2.16.6 #/1.12-2.16.4-2.8.1 py-neurodamus/2.16.4

rm *.log
rm *.out
rm *.SUCCESS

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json 
