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

#spack env activate bluerecording-dev
#source ~/bluerecording-env/bin/activate

module load unstable py-neurodamus neurodamus-neocortex

export NEURODAMUS_PYTHON=/gpfs/bbp.cscs.ch/project/proj83/tharayil/neurodamus
#export NEURODAMUS_PYTHON=../../../../neurodamus


srun dplace /gpfs/bbp.cscs.ch/project/proj83/tharayil/new_mod_file/with_neurodamus/x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
