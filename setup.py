from setuptools import setup

setup(
    name='bluerecording',
    version='0.0.2',    
    description='A tool for calculating extracellular recording lead fields',
    url='https://github.com/joseph-tharayil/create_lfp_weights_for_neurodamus',
    author='Blue Brain Project, EPFL',
    license='NONE!',
    packages=['bluerecording'],
    install_requires=['bluepysnap','mpi4py']
   )

