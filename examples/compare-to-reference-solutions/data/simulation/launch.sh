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
source ~/bluerecording-dev/bin/activate

export NEURODAMUS_PYTHON=/gpfs/bbp.cscs.ch/project/proj83/tharayil/neurodamus

echo $PYTHONPATH

neurodamus simulation_config.json

#srun dplace /gpfs/bbp.cscs.ch/project/proj85/scratch/from68/sonata_circuits/fullSSCx/longSim/compareModalities/f554be01-456c-4a15-8670-df39a3187b7e/0/x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
