// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace omni
{
namespace ev4sio_physx
{
struct IPhysx;
struct IPhysxVisualization;
} // namespace ev4sio_physx
} // namespace omni

namespace ovphysx
{
namespace internal
{
namespace sidecar
{

omni::ev4sio_physx::IPhysx* tryGetInjectedPhysxInterface();
omni::ev4sio_physx::IPhysxVisualization* tryGetInjectedPhysxVisualizationInterface();

} // namespace sidecar
} // namespace internal
} // namespace ovphysx
