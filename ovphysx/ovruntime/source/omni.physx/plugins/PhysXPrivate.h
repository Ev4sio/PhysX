// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/IPhysxPrivate.h>

namespace ev4sio_physx
{
class PxScene;
class PxCudaContextManager;
} // namespace ev4sio_physx

namespace omni
{
namespace ev4sio_physx
{

::ev4sio_physx::PxScene* privGetPhysXScene(omni::physics::AttachHandle attach);
void primGetRigidBodyInstancedData(omni::physics::AttachHandle attach, usdparser::ObjectId* ids, uint32_t numIds, InstancedData* data);
::ev4sio_physx::PxCudaContextManager* privGetCudaContextManager(omni::physics::AttachHandle attach, uint64_t sceneKey);

} // namespace ev4sio_physx
} // namespace omni
