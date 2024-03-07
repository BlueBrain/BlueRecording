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


rm -rf x86_64
rm -rf neocortex
rm -rf mod
rm -rf tmp
module purge

spack env activate neuron

#git clone --recursive git@bbpgitlab.epfl.ch:hpc/sim/models/neocortex.git
 
#cd neocortex
#./fetch_common.bash
  
module load unstable gcc intel-oneapi-compilers hpe-mpi
module load neurodamus-neocortex       # assuming you are building neocortex mod files. Otherwise, replace it with other model
   

# if you want to add new MOD files, you can now copy them to the respective directory. e.g.
#rm mod/v6/*
cp -r $NEURODAMUS_NEOCORTEX_ROOT/lib/mod mod
cp /gpfs/bbp.cscs.ch/project/proj83/tharayil/new_mod_file/mechanisms/* mod/ #v6/
    
# build special

build_neurodamus.sh mod

export NEURODAMUS_PYTHON=/gpfs/bbp.cscs.ch/project/proj83/tharayil/neurodamus


srun dplace ./x86_64/special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
