// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "ScArticulationCore.h"

#include "ScPhysics.h"
#include "ScBodyCore.h"
#include "ScBodySim.h"
#include "ScArticulationSim.h"

using namespace ev4sio_physx;

ev4sio_Sc::ArticulationCore::ArticulationCore() :
	mSim(NULL)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	mCore.solverIterationCounts		= 1<<8 | 4;
	mCore.sleepThreshold			= 5e-5f * scale.speed * scale.speed;
	mCore.freezeThreshold			= 5e-6f * scale.speed * scale.speed;
	mCore.wakeCounter				= Physics::sWakeCounterOnCreation;
	mCore.gpuRemapIndex				= 0xffffffff;
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

	if (mSim)
	{
		ev4sio_Dy::FeatherstoneArticulation* arti = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mSim->getLowLevelArticulation());
		arti->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER);
	}

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
		ev4sio_Dy::FeatherstoneArticulation* arti = static_cast<ev4sio_Dy::FeatherstoneArticulation*>(mSim->getLowLevelArticulation());
		arti->setGpuDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER);
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

void ev4sio_Sc::ArticulationCore::computeGeneralizedGravityForce(PxArticulationCache& cache, const bool rootMotion) const
{
	if(mSim)
		mSim->computeGeneralizedGravityForce(cache, rootMotion);
}

void ev4sio_Sc::ArticulationCore::computeCoriolisAndCentrifugalForce(PxArticulationCache& cache, const bool rootMotion) const
{
	if(mSim)
		mSim->computeCoriolisAndCentrifugalForce(cache, rootMotion);
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

void ev4sio_Sc::ArticulationCore::computeCoefficientMatrix(PxArticulationCache& cache) const
{
	if(mSim)
		mSim->computeCoefficientMatrix(cache);
}

bool ev4sio_Sc::ArticulationCore::computeLambda(PxArticulationCache& cache, PxArticulationCache& initialState, const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter) const
{
	return mSim ? mSim->computeLambda(cache, initialState, jointTorque, gravity, maxIter) : false;
}

void ev4sio_Sc::ArticulationCore::computeGeneralizedMassMatrix(PxArticulationCache& cache, const bool rootMotion) const
{
	if(mSim)
		mSim->computeGeneralizedMassMatrix(cache, rootMotion);
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

PxU32 ev4sio_Sc::ArticulationCore::getCoefficientMatrixSize() const
{
	return mSim ? mSim->getCoefficientMatrixSize() : 0xFFFFFFFFu;
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
