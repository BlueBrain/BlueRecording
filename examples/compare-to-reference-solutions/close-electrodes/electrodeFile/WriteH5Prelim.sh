#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod_small
#SBATCH --nodes=1
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=2:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj85
#SBATCH --no-requeue
#SBATCH --output=EEG_sonata_CoordsV.out
#SBATCH --error=EEG_sonata_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

module purge

source ~/envForReqGeneration/bin/activate

srun -n 1 python ../../../run_initialize_h5.py 'electrodes.csv' '../../../data/simulation/simulation_config.json' 'coeffs.h5' 
