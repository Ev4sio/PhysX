// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

#include <PhysXTools.h>
#include <OmniPhysX.h>

#include "PhysXPropertiesUpdate.h"

#include "internal/InternalMimicJoint.h"


using namespace ::ev4sio_physx;
using namespace carb;
using namespace omni::ev4sio_physx;
using namespace omni::ev4sio_physx::usdparser;
using namespace omni::ev4sio_physx::internal;


static const InternalDatabase::Record* getObjectRecord(const AttachedStage& attachedStage, omni::ev4sio_physx::PhysXType type,
    omni::ev4sio_physx::usdparser::ObjectId objectId)
{
    omni::ev4sio_physx::internal::InternalPhysXDatabase& db = attachedStage.getInternalPhysXDatabase();

    return db.getFullTypedRecord(type, objectId);
}

static InternalMimicJoint* getInternalMimicJoint(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::ObjectKey& key)
{
    const InternalDatabase::Record* objectRecord = getObjectRecord(attachedStage, ePTMimicJoint, objectId);
    if (objectRecord)
    {
        key = objectRecord->mKey;
        return static_cast<InternalMimicJoint*>(objectRecord->mInternalPtr);
    }
    else
        return nullptr;
}


bool omni::ev4sio_physx::updateMimicJointGearing(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setGearing(value);
    }

    return true;
}

bool omni::ev4sio_physx::updateMimicJointOffset(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setOffset(value);
    }

    return true;
}

bool omni::ev4sio_physx::updateMimicJointNaturalFrequency(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setNaturalFrequency(value);
    }

    return true;
}

bool omni::ev4sio_physx::updateMimicJointDampingRatio(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setDampingRatio(value);
    }

    return true;
}

// Newton: joint0 = coef0 + coef1 * joint1  maps to PhysX gearing = -coef1, offset = -coef0.
bool omni::ev4sio_physx::updateNewtonMimicJointCoef1(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setGearing(-value);
    }

    return true;
}

bool omni::ev4sio_physx::updateNewtonMimicJointCoef0(AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    omni::physics::parse::ObjectKey key;
    InternalMimicJoint* internalMimicJoint = getInternalMimicJoint(attachedStage, objectId, key);

    if (internalMimicJoint)
    {
        float value;
        if (!getValue<float>(attachedStage, key, property, timeCode, value))
            return true;

        internalMimicJoint->setOffset(-value);
    }

    return true;
}
