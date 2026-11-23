// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationCore.h"

#include "ScPhysics.h"
#include "ScArticulationSim.h"

using namespace ev4sio_physx;

ev4sio_Sc::ArticulationCore::ArticulationCore() : mSim(NULL)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	mCore.solverIterationCounts	= 1<<8 | 4;
	mCore.sleepThreshold		= 5e-5f * scale.speed * scale.speed;
	mCore.freezeThreshold		= 5e-6f * scale.speed * scale.speed;
	mCore.wakeCounter			= Physics::sWakeCounterOnCreation;
	mCore.gpuRemapIndex			= 0xffffffff;
}

ev4sio_Sc::ArticulationCore::~ArticulationCore()
{
}

//--------------------------------------------------------------
//
// ArticulationCore interface implementation
//
//--------------------------------------------------------------

void ev4sio_Sc::ArticulationCore::setWakeCounter(const PxReal v)
{
	mCore.wakeCounter = v;

	if (mSim)
	{
		mSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER);
	}

#if PX_DEBUG
	if(mSim)
		mSim->debugCheckWakeCounterOfLinks(v);
#endif
}

bool ev4sio_Sc::ArticulationCore::isSleeping() const
{
	return mSim ? mSim->isSleeping() : (mCore.wakeCounter == 0.0f);
}

void ev4sio_Sc::ArticulationCore::wakeUp(PxReal wakeCounter)
{
	mCore.wakeCounter = wakeCounter;

	if(mSim)
		mSim->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER);

#if PX_DEBUG
	if(mSim)
		mSim->debugCheckSleepStateOfLinks(false);
#endif
}

void ev4sio_Sc::ArticulationCore::putToSleep()
{
	mCore.wakeCounter = 0.0f;

	if (mSim)
	{
		// Call the ArticulationSim's putToSleep which zeros velocities
		mSim->putToSleep();
		
		mSim->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER);
	}

#if PX_DEBUG
	if(mSim)
		mSim->debugCheckSleepStateOfLinks(true);
#endif
}

void ev4sio_Sc::ArticulationCore::setArticulationFlags(PxArticulationFlags flags)
{
	mCore.flags = flags;
	if(mSim)
	{
		mSim->setArticulationDirty(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_USER_FLAGS);

		const bool isFixedBaseLink = flags & PxArticulationFlag::eFIX_BASE;
		mSim->setFixedBaseLink(isFixedBaseLink);
	}
}

PxU32 ev4sio_Sc::ArticulationCore::getDofs() const
{
	return mSim ? mSim->getDofs() : 0xFFFFFFFFu;
}

PxArticulationCache* ev4sio_Sc::ArticulationCore::createCache() const
{
	return mSim ? mSim->createCache() : NULL;
}

PxU32 ev4sio_Sc::ArticulationCore::getCacheDataSize() const
{
	return mSim ? mSim->getCacheDataSize() : 0xFFFFFFFFu;
}

void ev4sio_Sc::ArticulationCore::zeroCache(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->zeroCache(cache);
}

bool ev4sio_Sc::ArticulationCore::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const
{
	if(mSim)
		return mSim->applyCache(cache, flag);
	return false;
}

void ev4sio_Sc::ArticulationCore::copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, const bool isGpuSimEnabled) const
{
	if(mSim)
		mSim->copyInternalStateToCache(cache, flag, isGpuSimEnabled);
}

void ev4sio_Sc::ArticulationCore::packJointData(const PxReal* maximum, PxReal* reduced) const
{
	if(mSim)
		mSim->packJointData(maximum, reduced);
}

void ev4sio_Sc::ArticulationCore::unpackJointData(const PxReal* reduced, PxReal* maximum) const
{
	if(mSim)
		mSim->unpackJointData(reduced, maximum);
}

void ev4sio_Sc::ArticulationCore::commonInit() const
{
	if(mSim)
		mSim->commonInit();
}

void ev4sio_Sc::ArticulationCore::computeGeneralizedGravityForce(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeGeneralizedGravityForce(cache);
}

void ev4sio_Sc::ArticulationCore::computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeCoriolisAndCentrifugalForce(cache);
}

void ev4sio_Sc::ArticulationCore::computeGeneralizedExternalForce(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeGeneralizedExternalForce(cache);
}

void ev4sio_Sc::ArticulationCore::computeJointAcceleration(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeJointAcceleration(cache);
}

void ev4sio_Sc::ArticulationCore::computeJointForce(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeJointForce(cache);
}

void ev4sio_Sc::ArticulationCore::computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols) const
{
	if(mSim)
		mSim->computeDenseJacobian(cache, nRows, nCols);
}

void ev4sio_Sc::ArticulationCore::computeCoefficientMatrix_Deprecated(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeCoefficientMatrix_Deprecated(cache);
}

bool ev4sio_Sc::ArticulationCore::computeLambda_Deprecated(PxArticulationCache& cache, PxArticulationCache& initialState, const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter) const
{
	return mSim ? mSim->computeLambda_Deprecated(cache, initialState, jointTorque, gravity, maxIter) : false;
}

void ev4sio_Sc::ArticulationCore::computeGeneralizedMassMatrix(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeGeneralizedMassMatrix(cache);
}

PxVec3 ev4sio_Sc::ArticulationCore::computeArticulationCOM(const bool rootFrame) const
{
	return mSim ? mSim->computeArticulationCOM(rootFrame) : PxVec3(0.0f);
}

void ev4sio_Sc::ArticulationCore::computeCentroidalMomentumMatrix(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeCentroidalMomentumMatrix(cache);
}

PxU32 ev4sio_Sc::ArticulationCore::getCoefficientMatrixSize_Deprecated() const
{
	return mSim ? mSim->getCoefficientMatrixSize_Deprecated() : 0xFFFFFFFFu;
}

PxSpatialVelocity ev4sio_Sc::ArticulationCore::getLinkAcceleration(const PxU32 linkId, const bool isGpuSimEnabled) const
{
	return mSim ? mSim->getLinkAcceleration(linkId, isGpuSimEnabled) : PxSpatialVelocity();
}

PxU32 ev4sio_Sc::ArticulationCore::getGpuArticulationIndex() const
{
	return mSim ? mCore.gpuRemapIndex : 0xffffffff;
}

void ev4sio_Sc::ArticulationCore::updateKinematic(PxArticulationKinematicFlags flags)
{
	PX_ASSERT(mSim);

	if (mSim)
		mSim->updateKinematic(flags);
}

PxNodeIndex ev4sio_Sc::ArticulationCore::getIslandNodeIndex() const
{
	return mSim ? mSim->getIslandNodeIndex() : PxNodeIndex(PX_INVALID_NODE);
}

void ev4sio_Sc::ArticulationCore::setGlobalPose()
{
	if(mSim)
		mSim->setGlobalPose();
}
