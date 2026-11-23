// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// PhysX Compound Shape definition, helper class
namespace omni
{
namespace ev4sio_physx
{
class PhysXCompoundShape
{
public:
    PhysXCompoundShape()
    {
    }

    ~PhysXCompoundShape()
    {
    }

    const std::vector<::ev4sio_physx::PxShape*>& getShapes() const
    {
        return mShapes;
    }

private:
    std::vector<::ev4sio_physx::PxShape*> mShapes;
};
} // namespace ev4sio_physx
} // namespace omni
