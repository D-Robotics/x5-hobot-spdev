'''
// Copyright (c) 2024,D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
'''

import os
import sys
import subprocess

import setuptools
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

# option can be x86 or aarch64
arch="aarch64"

classifiers = ['Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 3.10',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

# Read the version from the VERSION file
with open(os.path.join(os.path.abspath(os.path.dirname(__file__)),
                    '../../VERSION')) as version_file:
    version = version_file.read().strip()

setup(
    name="hobot_dnn_rdkx5",
    version=version,
    author="d-robotics",
    author_email="technical_support@d-robotics.cc",
    description="python API for Deep Neural Network inference engine",
    classifiers = classifiers,
    package_dir = {'': './'},
    packages = ['hobot_dnn_rdkx5'],
    package_data = {'hobot_dnn_rdkx5': ['libdnnpy.so', 'pyeasy_dnn.so']},
    include_package_data = True,
    install_requires=[
        "numpy>=1.26.4"
    ],
)
