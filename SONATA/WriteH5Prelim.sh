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
#SBATCH --output=EEG_sonata_CoordsV.out
#SBATCH --error=EEG_sonata_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

module purge

mkdir eeg_sonata_new

source ~/probevenv/bin/activate


srun -n 1 python writeH5_prelim.py 'electrode_file.csv' 'EEG' 'S1FL' 'simulation_config.json' 'positionsO1_new' 'eeg_sonata_new' 50
