// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/Defines.h>

#include <cudamanager/PxCudaContextManager.h>
#include <foundation/PxSimpleTypes.h>
#include <foundation/PxTransform.h>
#include <solver/PxSolverDefs.h> // PxArticulationAxis, PxArticulationMotion

#include <PxArticulationFlag.h>
#include <PxArticulationTendon.h>
#include <PxArticulationTendonData.h>
#include <PxShape.h>
#include <PxDeformableMaterial.h>
#include <PxDeformableVolume.h>
#include <PxDeformableVolumeFlag.h>
#include <foundation/PxFlags.h>
#include <PxContact.h>

namespace ev4sio_physx
{
class PxArticulationReducedCoordinate;
class PxArticulationCache;
class PxArticulationLink;
class PxArticulationJointReducedCoordinate;
class PxArticulationFixedTendon;
class PxGpuFixedTendonData;
class PxArticulationSpatialTendon;
class PxGpuSpatialTendonData;
class PxRigidBody;
class PxRigidDynamic;
class PxShape;
class PxScene;
struct PxGpuContactPair;
} // namespace ev4sio_physx

namespace omni
{
namespace ev4sio_physx
{
namespace tensors
{

// how transforms are stored in public tensors - NOTE: it swaps p and q wrt PxTransform
struct TensorTransform
{
    ::ev4sio_physx::PxVec3 p;
    ::ev4sio_physx::PxQuat q;
};

// how velocities are stored in public tensors - NOTE: it swaps linear and angular wrt PhysX direct GPU API
struct TensorVelAcc
{
    ::ev4sio_physx::PxVec3 linear;
    ::ev4sio_physx::PxVec3 angular;
};

// how velocities are reported by PhysX direct GPU API (ev4sio_Cm::UnAlignedSpatialVector)
// TODO: can choose more general names for PhysxVelAcc and TensorTransform such as SO3LieGroupTuple and
// SO3LieAlgebraTuple to reuse the same structure for accelerations as well.
struct PhysxVelAcc
{
    ::ev4sio_physx::PxVec3 angular;
    ::ev4sio_physx::PxVec3 linear;
};

// how force sensor data are reported by PhysX direct GPU API (ev4sio_Cm::UnAlignedSpatialVector)
struct PhysxGpuSpatialForces
{
    ::ev4sio_physx::PxVec3 force;
    ::ev4sio_physx::PxVec3 torque;
};


class PhysxCudaContextGuard
{
public:
    explicit PhysxCudaContextGuard(::ev4sio_physx::PxCudaContextManager* cudaContextManager)
        : mCudaContextManager(cudaContextManager)
    {
        if (mCudaContextManager)
        {
            mCudaContextManager->acquireContext();
        }
    }

    ~PhysxCudaContextGuard()
    {
        if (mCudaContextManager)
        {
            mCudaContextManager->releaseContext();
        }
    }

private:
    ::ev4sio_physx::PxCudaContextManager* mCudaContextManager;
};

struct FreeD6RotationAxesFlag
{
    enum Enum
    {
        eTWIST = 1 << 0, //!< free rotation around X axis
        eSWING1 = 1 << 1, //!< free rotation around Y axis
        eSWING2 = 1 << 2 //!< free rotation around Z axis
    };
};

using FreeD6RotationAxesFlags = ::ev4sio_physx::PxFlags<FreeD6RotationAxesFlag::Enum, ::ev4sio_physx::PxU8>;

} // namespace tensors
} // namespace ev4sio_physx
} // namespace omni
