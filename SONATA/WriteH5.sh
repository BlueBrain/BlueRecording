#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=160
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

srun -n 4240 python writeH5.py 'EEG' '/gpfs/bbp.cscs.ch/project/proj68/scratch/tharayil/sonata_circuits/newVPM/testing/full/testVPM/newConfig/174b9760-77b7-47de-8008-ce817f046920/0/simulation_config.json' 'positions_hex0_new' 'eeg_sonata_new/coeffs.h5' 50 'electrode_file.csv' '/gpfs/bbp.cscs.ch/project/proj85/scratch/bbp_workflow/SSCx-O1-Calibrated-Workflow-5-12-22/81adc949-b514-4d78-853a-b369e4420dff/3/C6SurfNew.h5'
