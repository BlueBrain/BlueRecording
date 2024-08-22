# SPDX-License-Identifier: GPL-3.0-or-later
from setuptools import setup

setup(
    name='bluerecording',
    version='0.2.0',    
    description='A tool for calculating extracellular recording lead fields',
    url='https://github.com/BlueBrain/BlueRecording',
    author='Blue Brain Project, EPFL',
    license='GPL-3.0',
    packages=['bluerecording'],
    install_requires=[
    'bluepysnap',
    'libsonata>=0.1.28',
    'scikit-learn',
    'voxcell',
    'scipy',
    'numpy',
    'pandas',
    'morphio<3.3.7',
    'notebook',
    'ipython',
    'matplotlib',
    'MEAutility',
    'neuron',
    'connectome-utilities @ git+https://github.com/BlueBrain/ConnectomeUtilities.git#egg=connectome-utilities',
    'pytest-cov',
    'ipympl',
    'neo',
    'xarray',
    'allensdk @ git+https://github.com/AllenInstitute/AllenSDK',
    'cinplaAnalysis @ git+https://github.com/joseph-tharayil/CINPLA_Allen_V1_analysis'
    ]
   )

