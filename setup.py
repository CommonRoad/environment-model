import os
import re
import sys
import platform
import subprocess
from pathlib import Path

from setuptools import Extension
from setuptools.command.build_ext import build_ext
from distutils.core import setup
from distutils.version import LooseVersion


cmake_prefix = None
if '--cmake-prefix' in sys.argv:
    index = sys.argv.index('--cmake-prefix')
    sys.argv.pop(index)
    cmake_prefix = sys.argv.pop(index)


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.16':
                raise RuntimeError("CMake >= 3.16 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        if not cmake_prefix:
            raise RuntimeError("CMake Prefix is required for building this package (set using --cmake-prefix)")

        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = ["-DPYTHON_EXECUTABLE={}".format(sys.executable),
                      "-DCMAKE_PREFIX_PATH={}".format(cmake_prefix),
                      "-DINSTALL_GTEST=OFF",
                      "-DBUILD_TESTS=OFF",
                      "-DBUILD_DOXYGEN=OFF",
                      "-DBUILD_PYBIND=ON"]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j4']

        build_temp_dir = Path(self.build_temp)
        dist_dir = build_temp_dir / 'dist'
        build_dir =  build_temp_dir / 'build'
        lib_dir = dist_dir / 'lib'
        lib_python_dir = lib_dir / 'python'
        install_path = Path(self.get_ext_fullpath(ext.name))
        install_dir = install_path.parent

        for p in [dist_dir, build_dir, install_dir]:
            p.mkdir(exist_ok=True)

        cmake_args += [ '-DCMAKE_INSTALL_PREFIX:PATH={}'.format(dist_dir) ]

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--install', '.'] + build_args, cwd=build_dir)

        extension_file = lib_python_dir / install_path.name
        if not extension_file.exists():
            raise RuntimeError('Expected Python extension module \'{}\', but no such file exists'.format(extension_file))

        for file in lib_python_dir.iterdir():
            if file.suffix == '.so':
                self.copy_file(file, install_dir)

setup(
    name='cpp_env_model',
    version='0.0.1',
    url='https://commonroad.in.tum.de/',
    license='CC BY-NC-ND 4.0',
    author='Sebastian Maierhofer',
    author_email='sebastian.maierhofer@tum.de',
    description='CommonRoad C++ Environment Model',
    ext_modules=[CMakeExtension("cpp_env_model")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    install_requires=[
        'commonroad-io>=2021.1',
    ],
    extras_require={
        'tests': ['numpy>=1.20.0',
                  'pytest>=5.3.2', ],
    },
    include_package_data=True,
)
