// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// clang-format off
// clang-format on

#include "tensors/cpu/CpuDeformableMaterialView.h"
#include "tensors/cpu/CpuSimulationView.h"
#include <PxPhysicsAPI.h>
#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

using namespace ev4sio_physx;

namespace omni
{
namespace ev4sio_physx
{
namespace tensors
{


CpuDeformableMaterialView::CpuDeformableMaterialView(CpuSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries)
    : BaseDeformableMaterialView(sim, entries)
{
}

CpuDeformableMaterialView::~CpuDeformableMaterialView()
{
}

} // namespace tensors
} // namespace ev4sio_physx
} // namespace omni
