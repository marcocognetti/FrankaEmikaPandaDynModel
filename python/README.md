# Python module for the Panda robot dynamics

## Description
To compile the python module run the following command from the current folder:
```
python setup.py build_ext --inplace
```
It is required to have the C++ library **Eigen3** installed on the system. This will create on a Linux machine a file called `panda_dyn_model.cpython-38-x86_64-linux-gnu.so` which is the python module.

## Usage
To use it, you need to copy it to the folder where you want to use it and import it as follows:
```
import panda_dyn_model
```
An usage example is provided in the `example.py` file.