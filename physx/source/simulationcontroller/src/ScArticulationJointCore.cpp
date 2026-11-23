// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationJointCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationSim.h"
#include "ScArticulationJointSim.h"
#include "ScBodyCore.h"
#include "ScPhysics.h"

using namespace ev4sio_physx;

ev4sio_Sc::ArticulationJointCore::ArticulationJointCore(const PxTransform& parentFrame, const PxTransform& childFrame) :
	mCore			(parentFrame, childFrame),
	mSim			(NULL),
	mArticulation	(NULL),
	mRootType		(NULL),
	mLLLinkIndex	(0xffffffff)
{
}

ev4sio_Sc::ArticulationJointCore::~ArticulationJointCore()
{
	PX_ASSERT(getSim() == 0);
}

void ev4sio_Sc::ArticulationJointCore::setSimDirty()
{
	ev4sio_Sc::ArticulationJointSim* sim = getSim();
	if(sim)
		sim->setDirty();

	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
		artiSim->mJcalcDirty = true;
}

void ev4sio_Sc::ArticulationJointCore::setParentPose(const PxTransform& t)
{
	// AD: we check if it changed at all to avoid marking the complete articulation dirty for a jcalc.
	// The jcalc internally checks these ArticulationJointCoreDirtyFlag again so we would skip most things
	// but we'd still check all the joints. The same is also true for the following functions.
	if (!(mCore.parentPose == t))
	{
		mCore.parentPose = t;

		setDirty();
	}
}

void ev4sio_Sc::ArticulationJointCore::setChildPose(const PxTransform& t)
{
	if (!(mCore.childPose == t))
	{
		mCore.childPose = t;

		setDirty();
	}
}

void ev4sio_Sc::ArticulationJointCore::setTargetP(PxArticulationAxis::Enum axis, PxReal targetP)
{
	// this sets the target position in the core.
	mCore.targetP[axis] = targetP;

	// this sets the target position in the ll articulation. This needs to happen immediately because we might
	// look up the value using the cache API again, and that one is reading directly from the llArticulation.
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
	{
		ev4sio_Dy::ArticulationData& data = artiSim->getArticulationData();
		const ev4sio_Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		PxReal* jointTargetPositions = data.getJointTargetPositions();
		PxReal* jTargetPosition = &jointTargetPositions[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];

		if (dofId != 0xff)
		{
			jTargetPosition[dofId] = targetP;

			artiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS);
		}
	}

	// AD: does not need setDirty - we write directly into the llArticulation and the GPU part is 
	// handled by setArticulationDirty.
}

void ev4sio_Sc::ArticulationJointCore::setTargetV(PxArticulationAxis::Enum axis, PxReal targetV)
{
	// this sets the target velocity in the core.
	mCore.targetV[axis] = targetV;

	// this sets the target velocity in the ll articulation. This needs to happen immediately because we might
	// look up the value using the cache API again, and that one is reading directly from the llArticulation.
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
	{
		ev4sio_Dy::ArticulationData& data = artiSim->getArticulationData();
		const ev4sio_Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		PxReal* jointTargetVelocities = data.getJointTargetVelocities();
		PxReal* jTargetVelocity = &jointTargetVelocities[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];

		if (dofId != 0xff)
		{
			jTargetVelocity[dofId] = targetV;

			artiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL);
		}
	}

	// AD: does not need setDirty - we write directly into the llArticulation and the GPU part is 
	// handled by setArticulationDirty.
}

void ev4sio_Sc::ArticulationJointCore::setArmature(PxArticulationAxis::Enum axis, PxReal armature)
{
	if (mCore.armature[axis] != armature)
	{
		mCore.armature[axis] = armature;

		setSimDirty();
	}
}

void ev4sio_Sc::ArticulationJointCore::setJointPosition(PxArticulationAxis::Enum axis, const PxReal jointPos)
{
	// this sets the position in the core.
	mCore.jointPos[axis] = jointPos;

	// this sets the position in the ll articulation. This needs to happen immediately because we might
	// look up the value using the cache API again, and that one is reading directly from the llArticulation.
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
	{
		ev4sio_Dy::ArticulationData& data = artiSim->getArticulationData();
		const ev4sio_Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		PxReal* jointPositions = data.getJointPositions();
		PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];

		if (dofId != 0xff)
		{
			jPosition[dofId] = jointPos;

			artiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS);
		}
	}
}

// AD: we need this indirection right now because we could have updated joint vel using the cache, so
// the read from the joint core might be stale.
PxReal ev4sio_Sc::ArticulationJointCore::getJointPosition(PxArticulationAxis::Enum axis) const
{
	PxReal jointPos = mCore.jointPos[axis];
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
	{
		const ev4sio_Dy::ArticulationData& data = artiSim->getArticulationData();
		const ev4sio_Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		const PxReal* jointPositions = data.getJointPositions();
		const PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];

		if(dofId != 0xff)
			jointPos = jPosition[dofId];
	}

	return jointPos;
}

void ev4sio_Sc::ArticulationJointCore::setJointVelocity(PxArticulationAxis::Enum axis, const PxReal jointVel)
{
	mCore.jointVel[axis] = jointVel;

	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
	{
		ev4sio_Dy::ArticulationData& data = artiSim->getArticulationData();
		const ev4sio_Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		PxReal* jointVelocities = data.getJointVelocities();
		PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
		{
			jVelocity[dofId] = jointVel;

			artiSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES);
		}
	}
}

// AD: we need this indirection right now because we could have updated joint vel using the cache, so
// the read from the joint core might be stale.
PxReal ev4sio_Sc::ArticulationJointCore::getJointVelocity(PxArticulationAxis::Enum axis) const
{
	PxReal jointVel = mCore.jointVel[axis];
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->isLLArticulationInitialized())
	{
		const ev4sio_Dy::ArticulationData& data = artiSim->getArticulationData();
		const ev4sio_Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		const PxReal* jointVelocities = data.getJointVelocities();
		const PxReal* jVelocities = &jointVelocities[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
			jointVel = jVelocities[dofId];
	}

	return jointVel;
}

void ev4sio_Sc::ArticulationJointCore::setLimit(PxArticulationAxis::Enum axis, const PxArticulationLimit& limit)
{
	mCore.setLimit(axis, limit);
	
	setSimDirty();
}

void ev4sio_Sc::ArticulationJointCore::setDrive(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive)
{
	mCore.setDrive(axis, drive);
	
	setSimDirty();
}

void ev4sio_Sc::ArticulationJointCore::setFrictionParams(PxArticulationAxis::Enum axis, const PxJointFrictionParams& jointFrictionParams)
{
	mCore.setFrictionParams(axis, jointFrictionParams);

	setSimDirty();
}

void ev4sio_Sc::ArticulationJointCore::setMaxJointVelocity(PxArticulationAxis::Enum axis, PxReal maxJointV)
{
	mCore.setMaxJointVelocity(axis, maxJointV);

	setSimDirty();
}
