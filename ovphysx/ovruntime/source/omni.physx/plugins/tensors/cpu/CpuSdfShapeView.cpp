// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "tensors/cpu/CpuSdfShapeView.h"
#include "tensors/cpu/CpuSimulationView.h"

using namespace ev4sio_physx;

namespace omni
{
namespace ev4sio_physx
{
namespace tensors
{

CpuSdfShapeView::CpuSdfShapeView(CpuSimulationView* sim, const std::vector<SdfShapeEntry>& entries)
    : BaseSdfShapeView(sim, entries)
{
}

CpuSdfShapeView::~CpuSdfShapeView()
{
}

} // namespace tensors
} // namespace ev4sio_physx
} // namespace omni
