# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # Use ament convention but override ignore list to exclude style rules that conflict with Google style
    # Only enforce D413 (Missing blank line after last section) for better readability
    rc = main(argv=[
        '--ignore', 'D100', 'D101', 'D102', 'D103', 'D104', 'D105', 'D106', 'D107',
                   'D203', 'D212', 'D404', 'D213', 'D406', 'D407', 'D401',
        '.'
    ])
    assert rc == 0, 'Found code style errors / warnings'
