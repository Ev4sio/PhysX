// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-CONTROL-001
// @covers AC-1

// @implements REQ-CUSTOM-LIFETIME-001
// @covers AC-1 AC-2

// @implements REQ-ATTACH-IO-001
// @covers AC-3

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-9 AC-11 AC-12 AC-27
 */

// @implements REQ-LOAD-JOINTLIMIT-001
// @covers AC-8

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <carb/tasking/TaskingUtils.h>
#include <unordered_map>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysxCustomJoint.h>
#include <common/foundation/Allocator.h>

#include <private/omni/physx/PhysxUsd.h>
#include <physx/include/extensions/PxJointLimit.h>

namespace omni
{
namespace ev4sio_physx
{
namespace usdparser
{
class AttachedStage;
}

struct CustomJointInfo
{
    // Not a source-interned schema token: registerCustomJoint takes a bare
    // `const char*` with no IPhysicsSource to intern against, so this is a
    // plain string identifier incidentally shaped like a token.
    std::string jointPrimType;
    ICustomJointCallback customJointCb;
    ::ev4sio_physx::PxConstraintSolverPrep solverPrepFn;
    size_t jointDataSize;
    ::ev4sio_physx::PxU32 typeId;
    // Copies held by a parser snapshot or a live constraint pin registration.
    std::shared_ptr<unsigned char> lifetime;
};

class CustomPhysXJoint : public ::ev4sio_physx::PxConstraintConnector, public Allocateable
{
public:
    CustomPhysXJoint(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key,
                     const usdparser::CustomPhysxJointDesc& jointDesc,
                     ::ev4sio_physx::PxPhysics& physics,
                     const CustomJointInfo& jointInfo,
                     ::ev4sio_physx::PxConstraintFlag::Enum flags,
                     ::ev4sio_physx::PxRigidActor* actor0,
                     ::ev4sio_physx::PxRigidActor* actor1);

    void release();

    // PxConstraintConnector boilerplate
    void* prepareData();
    void onConstraintRelease();
    void onComShift(::ev4sio_physx::PxU32 actor);
    void onOriginShift(const ::ev4sio_physx::PxVec3& shift);
    void* getExternalReference(::ev4sio_physx::PxU32& typeID);

    ::ev4sio_physx::PxBase* getSerializable()
    {
        return NULL;
    }

    virtual ::ev4sio_physx::PxConstraintSolverPrep getPrep() const;
    virtual const void* getConstantBlock() const;
    virtual bool updatePvdProperties(::ev4sio_physx::pvdsdk::PvdDataStream& pvdConnection,
                                     const ::ev4sio_physx::PxConstraint* c,
                                     ::ev4sio_physx::PxPvdUpdateType::Enum updateType) const
    {
        return true;
    }
    virtual void updateOmniPvdProperties() const {};

    ~CustomPhysXJoint()
    {
    }

    void* callbackContext() const { return mCustomJointInfo.customJointCb.userData; }

    ::ev4sio_physx::PxConstraint* getConstraint()
    {
        return mConstraint;
    }

private:
    const void* wrapConstantBlock(const void* data) const;
    struct alignas(16) ShaderDataChunk { unsigned char bytes[16]; };
    mutable std::vector<ShaderDataChunk> mShaderData;
    const usdparser::AttachedStage& mAttachedStage;
    ::ev4sio_physx::PxConstraint* mConstraint;

    omni::physics::parse::ObjectKey mJointKey;
    CustomJointInfo mCustomJointInfo;
};

using CustomJointRegistryMap = std::unordered_map<size_t, CustomJointInfo>;
using CustomJointTypeMap = std::unordered_map<std::string, CustomJointInfo>;
using CustomJointMap = std::unordered_map<omni::physics::parse::ObjectKey, CustomPhysXJoint*, omni::physics::parse::ObjectKey::Hash>;

class PhysXCustomJointManager
{
public:
    PhysXCustomJointManager();
    ~PhysXCustomJointManager();

    size_t registerCustomJoint(const char* jointPrimType,
                               ICustomJointCallback& jointCallback,
                               ::ev4sio_physx::PxConstraintSolverPrep jointPrepFn,
                               size_t jointDataSize);
    void unregisterCustomJoint(size_t id);
    bool tryUnregisterCustomJoint(size_t id);
    void markJointDirty(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key);
    void setJointFlags(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key, CustomJointFlag::Enum flags);
    CustomJointFlag::Enum getJointFlags(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key);

    void clear();
    void clear(const usdparser::AttachedStage& owner);

    CustomPhysXJoint* createCustomJoint(const usdparser::AttachedStage& attachedStage,
                                        omni::physics::parse::ObjectKey primKey,
                                        const usdparser::CustomPhysxJointDesc& jointDesc,
                                        ::ev4sio_physx::PxRigidActor* actor0,
                                        const ::ev4sio_physx::PxTransform& localFrame0,
                                        ::ev4sio_physx::PxRigidActor* actor1,
                                        const ::ev4sio_physx::PxTransform& localFrame1);

    void removeCustomJoint(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey primKey, const CustomPhysXJoint* joint);

    CustomJointTypeMap getCustomJointTypeMap() const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        return mCustomJointTypeMap;
    }

private:
    size_t mJointRegistryCounter;
    CustomJointRegistryMap mCustomJointRegistryMap;
    CustomJointTypeMap mCustomJointTypeMap;
    mutable carb::tasking::MutexWrapper mRegistryMutex;
};


// Clamps both bounds of a revolute limit into [-180, 180] degrees, so a pair beyond one end, such
// as [200, 250] degrees, stays ordered and valid (REQ-LOAD-JOINTLIMIT-001 AC-8). Only a bound
// clearly outside that range is reported.
inline void checkRevoluteJointLimits(::ev4sio_physx::PxJointAngularLimitPair& limitPair, const char* primKey)
{
    const float ANGLE_CLAMP_THRESHOLD = ::ev4sio_physx::PxPi * 1.0001f;
    if (::ev4sio_physx::PxAbs(limitPair.lower) > ANGLE_CLAMP_THRESHOLD)
    {
        CARB_LOG_WARN("Lower angle limit on a revolute D6 joint was clamped to %s180 degrees. %s",
                      limitPair.lower < 0.0f ? "-" : "", primKey);
    }

    if (::ev4sio_physx::PxAbs(limitPair.upper) > ANGLE_CLAMP_THRESHOLD)
    {
        CARB_LOG_WARN("Upper angle limit on a revolute D6 joint was clamped to %s180 degrees. %s",
                      limitPair.upper < 0.0f ? "-" : "", primKey);
    }

    limitPair.lower = ::ev4sio_physx::PxClamp(limitPair.lower, -::ev4sio_physx::PxPi, ::ev4sio_physx::PxPi);
    limitPair.upper = ::ev4sio_physx::PxClamp(limitPair.upper, -::ev4sio_physx::PxPi, ::ev4sio_physx::PxPi);
}

} // namespace ev4sio_physx
} // namespace omni
