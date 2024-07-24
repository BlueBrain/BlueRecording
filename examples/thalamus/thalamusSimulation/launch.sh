#!/bin/bash -l
#SBATCH --account=proj85
#SBATCH --partition=prod
#SBATCH --nodes=32
#SBATCH --cpus-per-task=2
#SBATCH --mem=0
#SBATCH --constraint=cpu
#SBATCH --exclusive
#SBATCH --time=24:00:00
#SBATCH --job-name=CortexNrdmsPySim

# SPDX-License-Identifier: GPL-3.0-or-later


spack env activate bluerecording-thalamus
module use $SPACK_INSTALL_PREFIX/modules/linux-rhel7-skylake/
module load unstable neurodamus-thalamus/develop #neuron/develop py-neurodamus/develop

srun dplace special -mpi -python $NEURODAMUS_PYTHON/init.py --configFile=simulation_config.json --lb-mode=RoundRobin
