#!/bin/bash -l
#SBATCH --job-name="EEG_2_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=200
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj85
#SBATCH --no-requeue
#SBATCH --output=EEG_2_CoordsV.out
#SBATCH --error=EEG_2_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0


source ~/bluerecording-dev/bin/activate
spack env activate bluerecording-dev


srun -n 6000 python ../../../scripts/run_write_weights.py '../data/simulation/simulation_config.json' '../data/getPositions/positions_all_new' 'coeffsEcog_EEG.h5' 50 'ecog_eeg_2.csv' '/gpfs/bbp.cscs.ch/project/proj85/scratch/Forelimb_ECoG_2.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/Forelimb_EEG_2.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/Forelimb_LFP_2.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/Forelimb_ECoG_2.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/Forelimb_EEG_2.h5 /gpfs/bbp.cscs.ch/project/proj85/scratch/Forelimb_LFP_2.h5'
