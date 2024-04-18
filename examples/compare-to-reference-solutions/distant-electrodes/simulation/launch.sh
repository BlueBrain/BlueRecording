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

unset MODULEPATH
. /gpfs/bbp.cscs.ch/ssd/apps/bsd/pulls/2379/config/modules.sh
module load unstable neurodamus-neocortex

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json
