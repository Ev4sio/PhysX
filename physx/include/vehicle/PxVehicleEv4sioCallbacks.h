// Copyright (c) 2026 Ev4sio. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_VEHICLE_EV4SIO_CALLBACKS_H
#define PX_VEHICLE_EV4SIO_CALLBACKS_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

namespace ev4sio_physx
{
struct PxSweepHit;
class PxRigidActor;

/**
\brief Tire shader signature recovered from Ev4sio/PhysX-4.1.2 (76db61bb).
The extra longitudinal/lateral speeds are preserved. The default shader below
expects a PxVehicleTireForceParams pointer, using the current SDK's force units
and camber convention; it does not accept the removed PxVehicleTireData type.
*/
typedef void (*PxVehicleComputeTireForce)(const void* shaderData,
    PxF32 tireFriction, PxF32 longSlip, PxF32 latSlip, PxF32 camber,
    PxF32 tireLongSpeed, PxF32 tireLatSpeed,
    PxF32 wheelOmega, PxF32 wheelRadius, PxF32 recipWheelRadius,
    PxF32 restTireLoad, PxF32 normalisedTireLoad, PxF32 tireLoad,
    PxF32 gravity, PxF32 recipGravity,
    PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);

void ev4sio_PxVehicleComputeTireForceDefault(const void* shaderData,
    PxF32 tireFriction, PxF32 longSlip, PxF32 latSlip, PxF32 camber,
    PxF32 tireLongSpeed, PxF32 tireLatSpeed,
    PxF32 wheelOmega, PxF32 wheelRadius, PxF32 recipWheelRadius,
    PxF32 restTireLoad, PxF32 normalisedTireLoad, PxF32 tireLoad,
    PxF32 gravity, PxF32 recipGravity,
    PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);

/**
\brief Suspension sweep callback recovered from Ev4sio/PhysX-4.1.2 (2244410e).
Called once for a successful suspension sweep, after overlap recovery and before
the road plane is computed. wheelWidth is the full width, not halfWidth.
The callback may adjust the hit position, normal and distance; actor and shape
must remain valid. For an overlap recovery, the supplied start/direction describe
the recovery sweep. Callback pointers and user data are runtime-only state.
*/
typedef void (*PxVehicleModifySuspensionSweepHit)(const void* shaderData,
    const PxVec3& sweepStartPosition, const PxVec3& sweepDirection,
    PxF32 wheelWidth, PxF32 wheelRadius, PxSweepHit& sweepHit, PxRigidActor* hitActor);
}

#endif
