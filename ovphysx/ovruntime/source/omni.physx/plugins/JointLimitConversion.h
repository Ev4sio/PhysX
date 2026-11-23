// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-LOAD-JOINTLIMIT-001
// @covers AC-1 AC-2 AC-3 AC-5 AC-6

#pragma once

// Converts a joint limit bound, already in PhysX units (radians for angles), into the range the
// PhysX limit accepts. Joint creation and live limit updates both convert through these, so a
// bound reaches PhysX the same way on either path. An open bound (the -inf/inf schema fallback,
// or any value beyond it) becomes a value the joint cannot reach, so the authored bound of a
// one-sided limit still applies.

#include <PxPhysicsAPI.h>

namespace omni
{
namespace ev4sio_physx
{

// Standalone linear limit (PxJointLinearLimitPair). PhysX rejects a pair whose bounds or span are
// not finite, so clamp to its own default for an unlimited pair, +-PX_MAX_F32/3.
inline float toLinearLimitBound(const float bound)
{
    const float maxBound = PX_MAX_F32 / 3.0f;
    return ::ev4sio_physx::PxClamp(bound, -maxBound, maxBound);
}

// Standalone D6 twist limit (PxJointAngularLimitPair). PxD6Joint::setTwistLimit takes only angles
// strictly inside (-2pi, 2pi).
inline float toTwistLimitBound(const float bound)
{
    const float twoPiEpsilon = float(2.0 * 3.14159265358979323846 - 0.0000001);
    return ::ev4sio_physx::PxClamp(bound, -twoPiEpsilon, twoPiEpsilon);
}

// Standalone D6 swing limit (PxJointLimitPyramid). PhysX rejects a pyramid whose angles are not
// finite and strictly inside (-pi, pi).
inline float toPyramidSwingLimitBound(const float bound)
{
    const float piEpsilon = ::ev4sio_physx::PxPi - 1.0e-4f;
    return ::ev4sio_physx::PxClamp(bound, -piEpsilon, piEpsilon);
}

// Articulation joint limit (PxArticulationLimit). setLimitParams rejects non-finite limits, and a
// spherical joint (which D6 articulation joints are) takes only angles within +-pi.
inline float toArticulationLimitBound(const ::ev4sio_physx::PxArticulationJointType::Enum jointType, const float bound)
{
    const float maxBound =
        jointType == ::ev4sio_physx::PxArticulationJointType::eSPHERICAL ? ::ev4sio_physx::PxPi : PX_MAX_F32 / 3.0f;
    return ::ev4sio_physx::PxClamp(bound, -maxBound, maxBound);
}

} // namespace ev4sio_physx
} // namespace omni
