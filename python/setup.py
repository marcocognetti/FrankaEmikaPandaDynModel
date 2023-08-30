from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension

ext_modules = [
    Pybind11Extension("panda_dyn_model",
                      ["cpp_export.cpp", 
                       "../cpp/panda_dyn_model_example/src/CoriolisMatrix.cpp",
                       "../cpp/panda_dyn_model_example/src/FrictionTorque.cpp",
                       "../cpp/panda_dyn_model_example/src/GravityVector.cpp",
                       "../cpp/panda_dyn_model_example/src/MassMatrix.cpp"],
                      include_dirs=['/usr/include/eigen3',
                                    '../cpp/panda_dyn_model_example/include']),
]

setup(
    name="panda_dyn_model",
    ext_modules=ext_modules,
)
