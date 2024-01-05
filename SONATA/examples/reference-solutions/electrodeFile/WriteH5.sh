#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=1
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0


module purge


spack env activate writeCoefficientsEnv 

srun -n 1 python ../../../writeH5.py '../../data/simulation/simulation_config.json' '../../data/getPositions/positions' 'coeffs.h5' 50 'electrodes.csv' 'Infinite.h5 Infinite.h5' 
