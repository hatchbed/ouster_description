// Copyright (c) 2024, Marc Alban / Hatchbed LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Tests that all mesh files can be loaded by gz::common::MeshManager — the
// exact loader Gazebo uses at simulation startup.

#include <gtest/gtest.h>

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

static const std::string & MeshesDir()
{
  static const std::string dir =
    ament_index_cpp::get_package_share_directory("ouster_description") +
    "/meshes/";
  return dir;
}

class MeshLoadTest : public testing::TestWithParam<std::string> {};

TEST_P(MeshLoadTest, LoadsWithGazebo) {
  const std::string mesh_path = MeshesDir() + GetParam();
  const gz::common::Mesh *mesh =
    gz::common::MeshManager::Instance()->Load(mesh_path);
  ASSERT_NE(mesh, nullptr) << "MeshManager failed to load: " << mesh_path;
  EXPECT_GT(mesh->SubMeshCount(), 0u) << "No sub-meshes in: " << GetParam();
  for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i) {
    auto sub = mesh->SubMeshByIndex(i).lock();
    ASSERT_NE(sub, nullptr)
        << GetParam() << ": sub-mesh[" << i << "] lock failed";
    EXPECT_GT(sub->VertexCount(), 0u)
        << GetParam() << ": sub-mesh[" << i << "] has no vertices";
  }
}

INSTANTIATE_TEST_SUITE_P(OusterMeshFiles, MeshLoadTest,
                         testing::Values("os0_base.glb", "os0_fin_cap.glb",
                                         "os0_halo_cap.glb", "os0_sensor.glb",
                                         "os2_base.glb", "os2_sensor.glb",
                                         "osdome_sensor.glb"));
