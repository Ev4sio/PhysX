// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace omni
{
namespace ev4sio_physx
{

::ev4sio_physx::PxConvexMesh* createCylinderConvexMesh(const float width,
                                                const float radius,
                                                const uint32_t numCirclePoints,
                                                omni::ev4sio_physx::usdparser::Axis axis);

::ev4sio_physx::PxConvexMesh* createConeConvexMesh(const float width,
                                            const float radius,
                                            const uint32_t numCirclePoints,
                                            omni::ev4sio_physx::usdparser::Axis axis);

::ev4sio_physx::PxVec3 getConeOrCylinderScale(const ::ev4sio_physx::PxF32 halfWidth,
                                       const ::ev4sio_physx::PxF32 radius,
                                       omni::ev4sio_physx::usdparser::Axis axis);
void getConeOrCylinderSize(const ::ev4sio_physx::PxVec3& scale,
                           omni::ev4sio_physx::usdparser::Axis axis,
                           ::ev4sio_physx::PxF32& halfWidth,
                           ::ev4sio_physx::PxF32& radius);

} // namespace ev4sio_physx
} // namespace omni
