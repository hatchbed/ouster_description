#!/usr/bin/env python3
# Copyright (c) 2024, Marc Alban / Hatchbed LLC
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Unit tests for xacro expansion, URDF structure, and mesh file integrity."""
import functools
from pathlib import Path
import struct
import subprocess
import tempfile
import textwrap
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

_SHARE = Path(get_package_share_directory('ouster_description'))
_MESHES_DIR = _SHARE / 'meshes'

VARIANTS = [
    'OS0-32', 'OS0-64', 'OS0-128',
    'OS1-32', 'OS1-64', 'OS1-128',
    'OS2-32', 'OS2-64', 'OS2-128',
    'OSDome-32', 'OSDome-64', 'OSDome-128',
]

# Links and joints expected after instantiating any variant macro with
# name="sensor", lidar_link="lidar", imu_link="imu".
EXPECTED_LINKS = ('sensor_base_link', 'sensor', 'lidar', 'imu')
EXPECTED_JOINTS = (
    'sensor_mount_joint',
    'sensor_body_joint',
    'sensor_imu_link_joint',
    'sensor_lidar_link_joint',
)

_PACKAGE_MESH_PREFIX = 'package://ouster_description/meshes/'


@functools.lru_cache(maxsize=None)
def _expand_variant(variant: str) -> tuple:
    """
    Expand a sensor variant xacro and return (returncode, stdout, stderr).

    Wraps the variant in a minimal robot xacro that instantiates the macro
    with consistent test parameters. Results are cached so each variant is
    expanded exactly once per test session.
    """
    xacro_path = _SHARE / 'urdf' / f'{variant}.urdf.xacro'
    wrapper = textwrap.dedent(f"""\
        <?xml version="1.0"?>
        <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_robot">
          <xacro:include filename="{xacro_path}"/>
          <link name="base_link"/>
          <xacro:{variant} parent="base_link" name="sensor"
              lidar_link="lidar" imu_link="imu">
            <origin xyz="0 0 0" rpy="0 0 0"/>
          </xacro:{variant}>
        </robot>
    """)
    with tempfile.NamedTemporaryFile(
        suffix='.urdf.xacro', mode='w', delete=False
    ) as f:
        f.write(wrapper)
        tmp_path = f.name
    try:
        result = subprocess.run(
            ['xacro', tmp_path], capture_output=True, text=True
        )
        return result.returncode, result.stdout, result.stderr
    finally:
        Path(tmp_path).unlink(missing_ok=True)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('variant', VARIANTS)
def test_xacro_expands_without_error(variant):
    """Xacro must exit cleanly with no errors for every sensor variant."""
    returncode, _stdout, stderr = _expand_variant(variant)
    assert returncode == 0, (
        f'{variant}: xacro exited with code {returncode}:\n{stderr}'
    )


@pytest.mark.parametrize('variant', VARIANTS)
def test_urdf_is_valid_xml(variant):
    """The expanded URDF must be well-formed XML with a <robot> root element."""
    returncode, stdout, _stderr = _expand_variant(variant)
    assert returncode == 0, f'{variant}: xacro expansion failed'
    root = ET.fromstring(stdout)
    assert root.tag == 'robot', (
        f'{variant}: expected root tag "robot", got "{root.tag}"'
    )


@pytest.mark.parametrize('variant', VARIANTS)
def test_urdf_has_expected_links(variant):
    """All required links must be present after macro instantiation."""
    returncode, stdout, _stderr = _expand_variant(variant)
    assert returncode == 0, f'{variant}: xacro expansion failed'
    root = ET.fromstring(stdout)
    found = {el.get('name') for el in root.findall('link')}
    for link in EXPECTED_LINKS:
        assert link in found, (
            f'{variant}: link "{link}" missing; found {sorted(found)}'
        )


@pytest.mark.parametrize('variant', VARIANTS)
def test_urdf_has_expected_joints(variant):
    """All required joints must be present after macro instantiation."""
    returncode, stdout, _stderr = _expand_variant(variant)
    assert returncode == 0, f'{variant}: xacro expansion failed'
    root = ET.fromstring(stdout)
    found = {el.get('name') for el in root.findall('joint')}
    for joint in EXPECTED_JOINTS:
        assert joint in found, (
            f'{variant}: joint "{joint}" missing; found {sorted(found)}'
        )


@pytest.mark.parametrize('variant', VARIANTS)
def test_mesh_files_exist(variant):
    """Every package:// mesh URI in the expanded URDF must resolve to a real file."""
    returncode, stdout, _stderr = _expand_variant(variant)
    assert returncode == 0, f'{variant}: xacro expansion failed'
    root = ET.fromstring(stdout)

    mesh_files = []
    for mesh_el in root.iter('mesh'):
        filename = mesh_el.get('filename', '')
        if filename.startswith(_PACKAGE_MESH_PREFIX):
            rel = filename[len(_PACKAGE_MESH_PREFIX):]
            mesh_files.append(_MESHES_DIR / rel)

    assert len(mesh_files) > 0, f'{variant}: no <mesh> elements found in URDF'
    for path in mesh_files:
        assert path.exists(), (
            f'{variant}: mesh file not found on disk: {path}'
        )


@pytest.mark.parametrize('variant', VARIANTS)
def test_glb_mesh_files_have_valid_header(variant):
    """Every .glb mesh referenced in the URDF must have a valid glTF binary header."""
    returncode, stdout, _stderr = _expand_variant(variant)
    assert returncode == 0, f'{variant}: xacro expansion failed'
    root = ET.fromstring(stdout)

    glb_files = []
    for mesh_el in root.iter('mesh'):
        filename = mesh_el.get('filename', '')
        if filename.startswith(_PACKAGE_MESH_PREFIX) and filename.endswith('.glb'):
            rel = filename[len(_PACKAGE_MESH_PREFIX):]
            glb_files.append((_MESHES_DIR / rel, filename))

    for path, uri in glb_files:
        assert path.exists(), f'{variant}: GLB not found: {path}'
        with open(path, 'rb') as f:
            header = f.read(12)
        assert len(header) == 12, (
            f'{variant}: {uri}: file too short to be a GLB'
        )
        magic, version, _length = struct.unpack_from('<4sII', header)
        assert magic == b'glTF', (
            f'{variant}: {uri}: invalid GLB magic {magic!r}'
        )
        assert version == 2, (
            f'{variant}: {uri}: unexpected glTF version {version} (expected 2)'
        )
