# Running Neural Simulations with Neurodamus

In order to generate compartment reports to input to BlueRecording, and to use BlueRecording weights files to simulate extracellular recordings, neural simulations are run using  [neurodamus](https://github.com/BlueBrain/neurodamus)

## Installation of prerequisites

Assuming you are using an Ubuntu system, first, run

```
sudo apt-get install cmake libopenmpi-dev libhdf5-dev
```
Then, install NEURON with `python -m pip install NEURON-nightly`.

Install [libsonatareport](https://github.com/BlueBrain/libsonatareport) with

```
git clone https://github.com/BlueBrain/libsonatareport reports/src --recursive
cmake \
  -B reports/build \
  -S reports/src \
  -DCMAKE_INSTALL_PREFIX=reports/install \
  -DSONATA_REPORT_ENABLE_SUBMODULES=ON \
  -DSONATA_REPORT_ENABLE_TEST=OFF
cmake --build reports/build
cmake --install reports/build
```

## Installation of Neurodamus

Install the `new-conductance-source` branch of Neurodamus:
```
git clone https://github.com/BlueBrain/neurodamus.git
cd neurodamus
git checkout new-conductance-source
pip install .
```

## Installation of neurodamus-models

Clone [neurodamus-models](https://github.com/BlueBrain/neurodamus-models) and compile the appropriate mod files. In all of the examples provided for BlueRecording, the `neocortex` model is used. Thus, run 
```
git clone https://github.com/BlueBrain/neurodamus-models.git neurodamus-models/src
export CC=$(which mpicc)
export CXX=$(which mpicxx)
# This can be set directly if the installation location of neurodamus is known
DATADIR=$(python -c "import neurodamus; from pathlib import Path; print(Path(neurodamus.__file__).parent / 'data')")
cmake -B neurodamus-models/build -S neurodamus-models/src \
  -DCMAKE_INSTALL_PREFIX=$PWD/neurodamus-models/install \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
  -DCMAKE_PREFIX_PATH=$PWD/reports/install \
  -DNEURODAMUS_CORE_DIR=${DATADIR} \
  -DNEURODAMUS_MECHANISMS=neocortex
cmake --build neurodamus-models/build
cmake --install neurodamus-models/build
```

## Run neural simulations

To run neural simulations, use 
```
srun <srun params> <your_built_special> -mpi -python $NEURODAMUS_PYTHON/init.py <neurodamus params>
```
An example is provided [here](https://github.com/BlueBrain/BlueRecording/blob/main/examples/compare-to-reference-solutions/data/simulation/launch.sh)
