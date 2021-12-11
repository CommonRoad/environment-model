import sys

try:
    from skbuild import setup
except ImportError:
    print('Please update pip, you need pip 10 or greater,\n'
          ' or you need to install the PEP 518 requirements in pyproject.toml yourself', file=sys.stderr)
    raise


#from setuptools import find_packages

setup(
    name='commonroad-cpp',
    version='0.0.1',
    url='https://commonroad.in.tum.de/',
    license='CC BY-NC-ND 4.0',
    author='Sebastian Maierhofer',
    author_email='sebastian.maierhofer@tum.de',
    description='CommonRoad C++ Environment Model',
    install_requires=[
        'commonroad-io>=2021.3',
    ],
    extras_require={
        'tests': ['numpy>=1.20.0',
                  'pytest>=5.3.2', ],
    },
   # cmake_install_dir="src/commonroad_cpp",
  #  packages=find_packages(where="src"),
    cmake_args = ['-DINSTALL_GTEST:BOOL=OFF',
                  '-DENV_MODEL_BUILD_TESTS:BOOL=OFF',
                  '-DENV_MODEL_BUILD_DOXYGEN:BOOL=OFF',
                  '-DENV_MODEL_BUILD_PYBIND:BOOL=ON',
                  '-DENV_MODEL_BUILD_SHARED_LIBS:BOOL=ON',
                  '-DENV_MODEL_BUILD_EXECUTABLE:BOOL=OFF']
)
