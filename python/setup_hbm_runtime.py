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
               'License :: OSI Approved :: Apache License 2.0',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 3.10',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

# Read the version from the VERSION file
with open(os.path.join(os.path.abspath(os.path.dirname(__file__)),
                    '../../VERSION')) as version_file:
    version = version_file.read().strip()

setup(
    name="hbm_runtime",
    version=version,
    author="d-robotics",
    author_email="technical_support@d-robotics.cc",
    description="Python API for Horizon Bionic Motion Runtime",
    classifiers=classifiers,
    package_dir={'': './'},
    packages=['hbm_runtime'],
    package_data={'hbm_runtime': ['HB_HBMRuntime.so']},
    include_package_data=True,
)