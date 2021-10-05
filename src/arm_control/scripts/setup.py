from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules = cythonize(["arm_server_S4_V1_14.py"]))
