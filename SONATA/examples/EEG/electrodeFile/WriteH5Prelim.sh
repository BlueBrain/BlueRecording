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

source ~/envForReqGeneration/bin/activate


srun -n 1 python writeH5_prelim.py 'electrode_file.csv' 'EEG' '/gpfs/bbp.cscs.ch/project/proj68/scratch/tharayil/sonata_circuits/newVPM/testing/full/testVPM/newConfig/174b9760-77b7-47de-8008-ce817f046920/0/simulation_config.json' 'eeg_sonata_new/coeffs.h5' 
