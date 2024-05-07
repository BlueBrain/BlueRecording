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


export NEURODAMUS_PYTHON=/gpfs/bbp.cscs.ch/home/tharayil/spack_install/software/install_gcc-12.3.0-skylake/py-neurodamus-3.2.conductance-qaaluj/lib/python3.11/site-packages/neurodamus/data
#export  MOD_FILE_PATH=/gpfs/bbp.cscs.ch/home/tharayil/spack_install/software/install_oneapi-2023.2.0-skylake/neurodamus-models-42.42.42_conductance-hl4mep/share/neurodamus_neocortex/

#cp -r $MOD_FILE_PATH/mod .

#nrnivmodl mod

srun dplace ./x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
