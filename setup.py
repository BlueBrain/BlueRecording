from setuptools import setup

setup(
    name='bluerecording',
    version='0.0.3',    
    description='A tool for calculating extracellular recording lead fields',
    url='https://github.com/joseph-tharayil/create_lfp_weights_for_neurodamus',
    author='Blue Brain Project, EPFL',
    license='NONE!',
    packages=['bluerecording'],
    install_requires=[
    'bluepysnap',
    'scikit-learn',
    'voxcell',
    'scipy',
    'numpy',
    'morphio<3.3.7',
    'notebook',
    'ipython',
    'matplotlib',
    'MEAutility',
    'neuron',
    'connectome-utilities @ git+https://github.com/BlueBrain/ConnectomeUtilities.git#egg=connectome-utilities',
    'pytest-cov'
    ]
   )

