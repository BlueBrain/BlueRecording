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
#python test.py


module purge
spack env activate getPositionsEnv 

CHUNK_SIZE=50

NUMBER_OF_NEURONS=212000

NUMBER_OF_FILES=$(($NUMBER_OF_NEURONS/1000)) #Number of neurons in circuit divided by 1000

PATH_TO_SIMULATION_CONFIG='arbitraryPath'

POSITION_FOLDER_NAME='arbitraryName'

for i in {0..$NUMBER_OF_FILES}
do
    
    folder="$POSITION_FOLDER_NAME/$(($i/$CHUNK_SIZE))"
    mkdir -p $folder 2>/dev/null
    
done

srun -n $NUMBER_OF_FILES python getPositions.py $PATH_TO_SIMULATION_CONFIG $CHUNK_SIZE

