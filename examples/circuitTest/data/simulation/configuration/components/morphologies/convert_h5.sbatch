#!/bin/bash -l
#SBATCH --partition=prod
#SBATCH --account=proj83
#SBATCH --job-name ascii-to-h5
#SBATCH --nodes=1
#SBATCH --time=8:00:00
#SBATCH --exclusive
#SBATCH --constraint=cpu
#SBATCH --mem=0

module purge
module load archive/2021-05
module load py-morph-tool/2.4.4

morph-tool convert folder --extension h5 --ncores 32 $1 $2
