// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "carb/Defines.h"


namespace omni
{
namespace ev4sio_physx
{
namespace internal
{
struct InternalDebugDrawFlags
{
    enum Enum : uint32_t
    {
        eDEBUG_DRAW_SPLINES_SEGMENTS = 1 << 0,
        eDEBUG_DRAW_SPLINES = 2 << 0,
    };
};

}
} // namespace ev4sio_physx
} // namespace omni
