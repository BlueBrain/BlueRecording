#!/bin/bash -l
#SBATCH --job-name="EEG_1_CoordsV"
#SBATCH --partition=prod
#SBATCH --nodes=10
#SBATCH -C clx
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
##SBATCH --mail-type=ALL
#SBATCH --account=proj45
#SBATCH --no-requeue
#SBATCH --output=EEG_1_CoordsV.out
##SBATCH --error=EEG_1_CoordsV.err
#SBATCH --exclusive
#SBATCH --mem=0

module purge

spack env activate getPositionsEnv

FILES_PER_FOLDER=50

NUMBER_OF_NEURONS=212000

NEURONS_PER_FILE=1000

NUMBER_OF_FILES=$(($NUMBER_OF_NEURONS/$NEURONS_PER_FILE)) #Number of neurons in circuit divided by 1000

PATH_TO_SIMULATION_CONFIG='arbitraryPath'


POSITION_FOLDER_NAME='arbitraryName'

for i in {0..$NUMBER_OF_FILES}
do

    folder="$POSITION_FOLDER_NAME/$(($i/$FILES_PER_FOLDER))"
    mkdir -p $folder 2>/dev/null

done

srun -n $NUMBER_OF_FILES python getPositions.py $PATH_TO_SIMULATION_CONFIG $POSITION_FOLDER_NAME $NEURONS_PER_FILE $CHUNK_SIZE

