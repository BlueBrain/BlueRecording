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
#source ~/sirio/bin/activate

module load unstable py-bluepy py-mpi4py #hdf5 py-h5py

CHUNK_SIZE=50

for i in {0..212}
do
    
    folder="positionsO1_new/$(($i/$CHUNK_SIZE))"
    mkdir -p $folder 2>/dev/null
    
done

srun -n 212 python getPositionsRightV_2_O1.py ./BlueConfig $CHUNK_SIZE

#python getPositions2.py $SLURM_ARRAY_TASK_ID
#python combineFiles.py $SLURM_ARRAY_TASK_ID
#python combineCombos.py