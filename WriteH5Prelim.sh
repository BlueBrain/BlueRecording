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
#SBATCH --output=EEG_3_CoordsV.out
#SBATCH --error=EEG_3_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

module purge

mkdir neuropixels

source ~/envForReqGeneration_NonSonata/bin/activate
srun -n 1 python writeH5_prelim.py 'electrode_file.csv' 'LFP' './BlueConfig' 'positionsO1_new' 'electrodes/electrode.h5'
