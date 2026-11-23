// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationSim.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointCore.h"
#include "ScBodySim.h"
#include "ScArticulationTendonSim.h"
#include "ScArticulationMimicJointSim.h"
#include "ScScene.h"
#include "DyIslandManager.h"

using namespace ev4sio_physx;
using namespace ev4sio_Dy;
using namespace ev4sio_Sc;
using namespace ev4sio_IG;
using namespace ev4sio_Cm;

ArticulationSim* ev4sio_Sc::getArticulationSim(const IslandSim& islandSim, PxNodeIndex nodeIndex)
{
	return static_cast<ArticulationSim*>(getArticulationFromIG(islandSim, nodeIndex));
}

ev4sio_Sc::ArticulationSim::ArticulationSim(ArticulationCore& core, Scene& scene, BodyCore& root, ev4sio_Bp::AggregateHandle handle) :
	mScene						(scene),
	mCore						(core),
	mLinks						("ScArticulationSim::links"),
	mBodies						("ScArticulationSim::bodies"),
	mJoints						("ScArticulationSim::joints"),
	mAggregateHandle			(handle),
	mIsLLArticulationInitialized(false)
{
	mLinks.reserve(16);
	mJoints.reserve(16);
	mBodies.reserve(16);

	mIslandNodeIndex = scene.getSimpleIslandManager()->addNode(false, false, ev4sio_IG::Node::eARTICULATION_TYPE, this);

	PX_ASSERT(root.getSim());

	addBody(*root.getSim(), NULL, NULL);

	mCore.setSim(this);

	initData(&core.getCore(), NULL);
}

ev4sio_Sc::ArticulationSim::~ArticulationSim()
{
	mScene.getSimpleIslandManager()->removeNode(mIslandNodeIndex);

	mCore.setSim(NULL);
}

PxU32 ev4sio_Sc::ArticulationSim::findBodyIndex(BodySim& body) const
{
	for(PxU32 i=0; i<mBodies.size(); i++)
	{
		if(mBodies[i]==&body)
			return i;
	}
	PX_ASSERT(0);
	return 0x80000000;
}

void ev4sio_Sc::ArticulationSim::addLoopConstraint(ConstraintSim* constraintSim)
{
	const PxU32 size = mLoopConstraints.size();
	if (size < mLoopConstraints.size())
		mLoopConstraints.reserve(size*2 + 1);
	
	BodySim* bodySim0 = constraintSim->getBody(0);
	BodySim* bodySim1 = constraintSim->getBody(1);

	ArticulationLoopConstraint lConstraint;
	if (bodySim0)
		lConstraint.linkIndex0 = findBodyIndex(*bodySim0);
	else
		lConstraint.linkIndex0 = 0x80000000;

	if(bodySim1)
		lConstraint.linkIndex1 = findBodyIndex(*bodySim1);
	else
		lConstraint.linkIndex1 = 0x80000000;

	lConstraint.constraint = &constraintSim->getLowLevelConstraint();

	mLoopConstraints.pushBack(lConstraint);
}

void ev4sio_Sc::ArticulationSim::removeLoopConstraint(ConstraintSim* constraintSim)
{
	ev4sio_Dy::Constraint* constraint = &constraintSim->getLowLevelConstraint();

	const PxU32 size = mLoopConstraints.size();
	PxU32 index = 0;
	while (index < size && mLoopConstraints[index].constraint != constraint)
		++index;

	if (index != size)
		mLoopConstraints.replaceWithLast(index);
}

void ev4sio_Sc::ArticulationSim::addBody(BodySim& body, BodySim* parent, ArticulationJointSim* joint)
{
	mBodies.pushBack(&body);
	mJoints.pushBack(joint);
	FeatherstoneArticulation::addBody();

	const PxU32 index = mLinks.size();

	PX_ASSERT((((index==0) && (joint == 0)) && (parent == 0)) ||
			  (((index!=0) && joint) && (parent && (parent->getArticulation() == this))));

	ArticulationLink& link = *mLinks.insert();

	link.initBody(&body.getBodyCore().getCore());

	bool shouldSleep;
	bool currentlyAsleep;
	const bool bodyReadyForSleep = body.checkSleepReadinessBesidesWakeCounter();
	const PxReal wakeCounter = getCore().getWakeCounter();

	if(parent)
	{
		currentlyAsleep = !mBodies[0]->isActive();
		shouldSleep = currentlyAsleep && bodyReadyForSleep;

		const PxU32 parentIndex = findBodyIndex(*parent);
		link.initJoint(&joint->getCore().getCore(), parentIndex);
		
		ArticulationLink& parentLink = mLinks[parentIndex];
		if (parentLink.mChildrenStartIndex == 0xffffffff)
			parentLink.mChildrenStartIndex = index;

		parentLink.mNumChildren++;
	}
	else
	{
		link.initJoint(NULL, DY_ARTICULATION_LINK_NONE);

		currentlyAsleep = (wakeCounter == 0.0f);
		shouldSleep = currentlyAsleep && bodyReadyForSleep;
	}
	
	if(currentlyAsleep && !shouldSleep)
	{
		for(PxU32 i=0; i < (mBodies.size() - 1); i++)
			mBodies[i]->internalWakeUpArticulationLink(wakeCounter);
	}

	body.setArticulation(this, wakeCounter, shouldSleep, index);
}

void ev4sio_Sc::ArticulationSim::removeBody(BodySim& body)
{
	for (PxU32 i = 0; i < mBodies.size(); ++i)
	{
		if (mBodies[i] == &body)
		{
			mBodies.replaceWithLast(i);
			mJoints.replaceWithLast(i);
			break;
		}
	}
}

void ev4sio_Sc::ArticulationSim::addTendon(ArticulationSpatialTendonSim* const tendonSim)
{
	tendonSim->mArtiSim = this;

	const PxU32 index = mSpatialTendons.size();
	ev4sio_Dy::ArticulationSpatialTendon& llTendon = tendonSim->mLLTendon;
	llTendon.setTendonIndex(index);
	mSpatialTendons.pushBack(&llTendon);

	//mSpatialTendons.pushBack(&tendonSim->mLLTendon);
}

void ev4sio_Sc::ArticulationSim::addTendon(ArticulationFixedTendonSim* const tendonSim)
{
	tendonSim->mArtiSim = this;
	
	const PxU32 index = mFixedTendons.size();
	ev4sio_Dy::ArticulationFixedTendon& llTendon = tendonSim->mLLTendon;
	llTendon.setTendonIndex(index);
	mFixedTendons.pushBack(&llTendon);
}

void ev4sio_Sc::ArticulationSim::addMimicJoint(ArticulationMimicJointSim* const mimicJointSim, const PxU32 linkA, const PxU32 linkB)
{
	const PxU32 index = mMimicJoints.size();
	mimicJointSim->setLowLevelIndex(index);
	mimicJointSim->mArticulationSim = this;

	ev4sio_Dy::ArticulationMimicJointCore& llMimicJoint = mimicJointSim->getLLMimicJoint();
	llMimicJoint.linkA = linkA;
	llMimicJoint.linkB = linkB;
	mMimicJoints.pushBack(&llMimicJoint);
}

void ev4sio_Sc::ArticulationSim::createLLStructure()
{
	if(!mBodies.size())
		return;

	setupLinks(mLinks.size(), mLinks.begin(),
		(!mScene.isUsingGpuDynamics()) && (mScene.getSolverType() == PxSolverType::eTGS) && (mScene.getFlags() & PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS));

	assignTendons(mSpatialTendons.size(), mSpatialTendons.begin());

	assignTendons(mFixedTendons.size(), mFixedTendons.begin());
	
	assignMimicJoints(mMimicJoints.size(), mMimicJoints.begin());

	mIsLLArticulationInitialized = true;
}

void ev4sio_Sc::ArticulationSim::initializeConfiguration()
{
	ev4sio_Dy::ArticulationData& data = getArticulationData();
	jcalc(data);
	mJcalcDirty = false;

	const ev4sio_Dy::ArticulationLink* PX_RESTRICT links = data.getLinks();
	const ev4sio_Dy::ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();
	const PxU32 linkCount = data.getLinkCount();

	PxReal* PX_RESTRICT jointVelocites = data.getJointVelocities();
	PxReal* PX_RESTRICT jointPositions = data.getJointPositions();
	PxReal* PX_RESTRICT jointTargetPositions = data.getJointTargetPositions();
	PxReal* PX_RESTRICT jointTargetVelocities = data.getJointTargetVelocities();
	
	for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
	{
		const ev4sio_Dy::ArticulationLink& link = links[linkID];

		const ev4sio_Dy::ArticulationJointCore* joint = link.inboundJoint;
		const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[linkID];

		PxReal* jPositions = &jointPositions[jointDatum.jointOffset];
		PxReal* jVelocites = &jointVelocites[jointDatum.jointOffset];
		PxReal* jTargetPositions = &jointTargetPositions[jointDatum.jointOffset];
		PxReal* jTargetVelocities = &jointTargetVelocities[jointDatum.jointOffset];

		for (PxU8 i = 0; i < jointDatum.nbDof; ++i)
		{
			const PxU32 dofId = joint->dofIds[i];
			jPositions[i] = joint->jointPos[dofId];
			jVelocites[i] = joint->jointVel[dofId];
			jTargetPositions[i] = joint->targetP[dofId];
			jTargetVelocities[i] = joint->targetV[dofId];
		}
	}

	PxU32 flags = (ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS |
				  ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES |
				  ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS |
				  ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL);

	raiseGPUDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::Enum(flags));

	initPathToRoot();
}

void ev4sio_Sc::ArticulationSim::updateKinematic(PxArticulationKinematicFlags flags)
{
	ev4sio_Dy::ArticulationData& data = getArticulationData();
	if (mJcalcDirty)
	{
		jcalc(data);
		mJcalcDirty = false;
	}

	if ((flags & PxArticulationKinematicFlag::ePOSITION))
	{
		raiseGPUDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS);
		teleportLinks(data);
	}

	if ((flags & PxArticulationKinematicFlag::ePOSITION) ||
		(flags & PxArticulationKinematicFlag::eVELOCITY))
	{
		raiseGPUDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES);
		computeLinkVelocities(data);
	}
}

void ev4sio_Sc::ArticulationSim::copyJointStatus(const PxU32 linkID)
{
	const ev4sio_Dy::ArticulationData& data = getArticulationData();
	const ev4sio_Dy::ArticulationLink* PX_RESTRICT links = data.getLinks();
	const ev4sio_Dy::ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();

	const ev4sio_Dy::ArticulationLink& link = links[linkID];
	ev4sio_Dy::ArticulationJointCore* PX_RESTRICT joint = link.inboundJoint;
	const ev4sio_Dy::ArticulationJointCoreData& jointDatum = jointData[linkID];

	const PxReal* PX_RESTRICT jointVelocites = data.getJointVelocities();
	const PxReal* PX_RESTRICT jointPositions = data.getJointPositions();
	
	const PxReal* PX_RESTRICT jVelocities = &jointVelocites[jointDatum.jointOffset];
	const PxReal* PX_RESTRICT jPositions = &jointPositions[jointDatum.jointOffset];

	for(PxU8 i = 0; i < jointDatum.nbDof; ++i)
	{
		const PxU32 dofId = joint->dofIds[i];
		joint->jointPos[dofId] = jPositions[i];
		joint->jointVel[dofId] = jVelocities[i];
	}
}

void ev4sio_Sc::ArticulationSim::putToSleep()
{
	for (PxU32 i = 0; i < mLinks.size(); i++)
	{
		BodySim* bodySim = mBodies[i];
		PxsRigidBody& rigid = bodySim->getLowLevelBody();
		
		PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();

		//rigid.setPose(rigid.getLastCCDTransform());

		//KS - the IG deactivates bodies in parallel with the solver. It appears that under certain circumstances, the solver's integration (which performs
		//sleep checks) could decide that the body is no longer a candidate for sleeping on the same frame that the island gen decides to deactivate the island
		//that the body is contained in. This is a rare occurrence but the behavior we want to emulate is that of IG running before solver so we should therefore
		//permit the IG to make the authoritative decision over whether the body should be active or inactive.
		bodyCore.wakeCounter = 0.0f;
		bodyCore.linearVelocity = PxVec3(0.0f);
		bodyCore.angularVelocity = PxVec3(0.0f);

		rigid.clearAllFrameFlags();

		//Force update
	}
	
	// Zero joint velocities as documented in PxArticulationReducedCoordinate::putToSleep()
	// First zero them in the ev4sio_Sc::ArticulationJointCore objects via the joints array
	// This is what getJointVelocity() reads from when LL articulation is not initialized
	for (PxU32 i = 1; i < mJoints.size(); ++i)  // Start from 1 since root has no joint
	{
		ArticulationJointSim* jointSim = mJoints[i];
		if (jointSim)
		{
			ev4sio_Sc::ArticulationJointCore& jointCore = jointSim->getCore();
			// Zero all the cached joint velocities in the Sc layer
			for (PxU32 axis = 0; axis < PxArticulationAxis::eCOUNT; ++axis)
			{
				jointCore.getCore().jointVel[axis] = 0.0f;
			}
		}
	}
	
	// Only access low-level articulation data if it's initialized

	if (mIsLLArticulationInitialized)
	{
		ev4sio_Dy::ArticulationData& articulationData = getArticulationData();
		const PxU32 dofs = articulationData.getDofs();
		if (dofs > 0)
		{
			const PxU32 jointDataSize = sizeof(PxReal) * dofs;

			// Zero the joint velocities
			PxReal* jointVelocities = articulationData.getJointVelocities();
			if (jointVelocities)
				PxMemZero(jointVelocities, jointDataSize);

			// Also zero the new joint velocities (used for solver updates)
			PxReal* jointNewVelocities = articulationData.getJointNewVelocities();
			if (jointNewVelocities)
				PxMemZero(jointNewVelocities, jointDataSize);

			// Also zero position iteration joint velocities to be thorough
			PxReal* posIterJointVelocities = articulationData.getPosIterJointVelocities();
			if (posIterJointVelocities)
				PxMemZero(posIterJointVelocities, jointDataSize);
		}
	}
	
	mScene.getSimulationController()->updateArticulation(this, mIslandNodeIndex);
}

void ev4sio_Sc::ArticulationSim::sleepCheck(PxReal dt, PxMutex& articulationSleepLock)
{
	if(!mBodies.size())
		return;

#if PX_CHECKED
	{
		PxReal maxTimer = 0.0f, minTimer = PX_MAX_F32;
		bool allActive = true, noneActive = true;
		PX_UNUSED(allActive);
		PX_UNUSED(noneActive);
		for(PxU32 i=0;i<mLinks.size();i++)
		{
			PxReal timer = mBodies[i]->getBodyCore().getWakeCounter();
			maxTimer = PxMax(maxTimer, timer);
			minTimer = PxMin(minTimer, timer);
			bool active = mBodies[i]->isActive();
			allActive &= active;
			noneActive &= !active;
		}
		// either all links are asleep, or no links are asleep
		PX_ASSERT(maxTimer==0 || minTimer!=0);
		PX_ASSERT(allActive || noneActive);
	}

#endif

	if(!mBodies[0]->isActive())
		return;

	const PxReal sleepThreshold = getCore().getCore().sleepThreshold;

	PxReal maxTimer = 0.0f , minTimer = PX_MAX_F32;

	const PxU32 nbLinks = mLinks.size();
	for(PxU32 i=0; i<nbLinks; i++)
	{
		const ev4sio_Cm::SpatialVector& motionVelocity = getMotionVelocity(i);
		const PxReal timer = mBodies[i]->updateWakeCounter(dt, sleepThreshold, motionVelocity);
		maxTimer = PxMax(maxTimer, timer);
		minTimer = PxMin(minTimer, timer);
	}

	mCore.setWakeCounterInternal(maxTimer);

	if(maxTimer != 0.0f)
	{
		if(minTimer == 0.0f)
		{
			// make sure nothing goes to sleep unless everything does
			for(PxU32 i=0; i<nbLinks; i++)
				mBodies[i]->getBodyCore().setWakeCounterFromSim(PxMax(1e-6f, mBodies[i]->getBodyCore().getWakeCounter()));
		}
		return;
	}

	// PT: we reach this when the articulation goes to sleep

	for(PxU32 i=0; i<nbLinks; i++)
	{
		//mBodies[i]->notifyReadyForSleeping();	// PT: TODO: this seems unnecessary
		mBodies[i]->getLowLevelBody().resetSleepFilter();
	}

	// /PT: we are calling this function from multiple threads but deactivateNode() is not thread-safe. Using a lock as an initial fix.
	articulationSleepLock.lock();
	mScene.getSimpleIslandManager()->deactivateNode(mIslandNodeIndex);
	articulationSleepLock.unlock();
}

bool ev4sio_Sc::ArticulationSim::isSleeping() const
{
	return (mBodies.size() > 0) ? (!mBodies[0]->isActive()) : true;
}

void ev4sio_Sc::ArticulationSim::internalWakeUp(PxReal wakeCounter)
{
	if(mCore.getWakeCounter() < wakeCounter)
	{
		mCore.setWakeCounterInternal(wakeCounter);
		for(PxU32 i=0;i<mBodies.size();i++)
			mBodies[i]->internalWakeUpArticulationLink(wakeCounter);
	}
}

void ev4sio_Sc::ArticulationSim::updateForces(PxReal dt)
{
	PxU32 count = 0;
	bool anyForcesApplied = false;

	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i+1],128);
			PxPrefetchLine(mBodies[i+1],256);
		}

		ev4sio_Cm::SpatialVector* accelerations = mAcceleration.begin();
		anyForcesApplied |= mBodies[i]->updateForces(dt, NULL, NULL, count, &accelerations[i], NULL);
	}
	if(anyForcesApplied)
		raiseGPUDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL);
}

void ev4sio_Sc::ArticulationSim::clearAcceleration(PxReal dt)
{
	PxU32 count = 0;

	bool anyBodyRetains = false;
	
	for (PxU32 i = 0; i < mBodies.size(); i++)
	{
		if (i + 1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i + 1], 128);
			PxPrefetchLine(mBodies[i + 1], 256);
		}

		const bool accDirty = mBodies[i]->readVelocityModFlag(VMF_ACC_DIRTY);

		// the code restores the pre-impulse state:
		// if we only applied an impulse and no acceleration, we clear the acceleration here.
		// if we applied an acceleration, we re-apply the acceleration terms we have in the velMod.
		// we cleared out the impulse here when we pushed the data at the start of the sim.

		ev4sio_Cm::SpatialVector* accelerations = mAcceleration.begin();
		if (!accDirty)
		{
			accelerations[i].linear = PxVec3(0.f);
			accelerations[i].angular = PxVec3(0.f);
		}
		else
		{
			mBodies[i]->updateForces(dt, NULL, NULL, count, &accelerations[i], NULL);
		}

		// we need to raise the dirty flag if retain accelerations is on
		// because in that case we need to restore the acceleration without impulses. We
		// can only do that using the CPU->GPU codepath because we don't distinguish between
		// acceleration and impulses on the GPU.
		// The flag must be raised here because we don't know at the start of the next sim step
		// that the data in velMod is actually valid and the articulation would not be added
		// to the dirty list.

		// without retain accelerations, the accelerations are cleared directly on the GPU.
		if (mBodies[i]->getFlagsFast() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS)
			anyBodyRetains = true;
	}
	
	if (anyBodyRetains)
	{
		mScene.getSimulationController()->updateArticulationExtAccel(this, mIslandNodeIndex);
	}
}

void ev4sio_Sc::ArticulationSim::saveLastCCDTransform()
{
	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			PxPrefetchLine(mBodies[i+1],128);
			PxPrefetchLine(mBodies[i+1],256);
		}
		mBodies[i]->getLowLevelBody().saveLastCCDTransform();
	}
}

void ev4sio_Sc::ArticulationSim::setFixedBaseLink(bool value)
{
	const PxU32 linkCount = mLinks.size();

	if(linkCount > 0)
		mLinks[0].bodyCore->fixedBaseLink = PxU8(value);
}

PX_COMPILE_TIME_ASSERT(sizeof(ev4sio_Cm::SpatialVector)==sizeof(PxSpatialForce));
PxArticulationCache* ev4sio_Sc::ArticulationSim::createCache()
{
	return FeatherstoneArticulation::createCache(getDofs(), mLinks.size());
}

PxU32 ev4sio_Sc::ArticulationSim::getCacheDataSize() const
{
	return FeatherstoneArticulation::getCacheDataSize(getDofs(), mLinks.size());
}

void ev4sio_Sc::ArticulationSim::zeroCache(PxArticulationCache& cache) const
{
	const PxU32 cacheDataSize = getCacheDataSize();

	PxMemZero(cache.externalForces, cacheDataSize);
}

//copy external data to internal data
bool ev4sio_Sc::ArticulationSim::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag)
{
	//checkResize();
	bool shouldWake = false;
	if (FeatherstoneArticulation::applyCache(cache, flag, shouldWake))
	{
		mScene.getSimulationController()->updateArticulation(this, mIslandNodeIndex);
	}
	return shouldWake;
}

void ev4sio_Sc::ArticulationSim::computeGeneralizedGravityForce(PxArticulationCache& cache)
{
	getGeneralizedGravityForce(mScene.getGravity(), cache);
}

void ev4sio_Sc::ArticulationSim::computeJointAcceleration(PxArticulationCache& cache)
{
	getJointAcceleration(mScene.getGravity(), cache);
}

bool ev4sio_Sc::ArticulationSim::computeLambda_Deprecated(PxArticulationCache& cache, PxArticulationCache& initialState,
	const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter)
{
	const PxReal invLengthScale = 1.f / mScene.getLengthScale();
	return getLambda_Deprecated(mScene.getDynamicsContext()->getConstraintWriteBackPool(), mLoopConstraints.begin(), mLoopConstraints.size(), cache, initialState, jointTorque, gravity, maxIter, invLengthScale);
}

PxU32 ev4sio_Sc::ArticulationSim::getCoefficientMatrixSize_Deprecated() const
{
	const PxU32 size = mLoopConstraints.size();
	const PxU32 totalDofs = getDofs();
	return size * totalDofs;
}

void ev4sio_Sc::ArticulationSim::setJointDirty(ev4sio_Dy::ArticulationJointCore& jointCore)
{
	PX_UNUSED(jointCore);
	mScene.getSimulationController()->updateArticulationJoint(this, mIslandNodeIndex);
}

void ev4sio_Sc::ArticulationSim::setArticulationDirty(PxU32 flag)
{
	raiseGPUDirtyFlag(ev4sio_Dy::ArticulationDirtyFlag::Enum(flag));
	mScene.getSimulationController()->updateArticulation(this, mIslandNodeIndex);
}

void ev4sio_Sc::ArticulationSim::debugCheckWakeCounterOfLinks(PxReal wakeCounter) const
{
	PX_UNUSED(wakeCounter);

#if PX_DEBUG
	// make sure the links are in sync with the articulation
	for(PxU32 i=0; i < mBodies.size(); i++)
	{
		PX_ASSERT(mBodies[i]->getBodyCore().getWakeCounter() == wakeCounter);
	}
#endif
}

void ev4sio_Sc::ArticulationSim::debugCheckSleepStateOfLinks(bool isSleeping) const
{
	PX_UNUSED(isSleeping);

#if PX_DEBUG
	// make sure the links are in sync with the articulation
	for(PxU32 i=0; i < mBodies.size(); i++)
	{
		if (isSleeping)
		{
			PX_ASSERT(!mBodies[i]->isActive());
			PX_ASSERT(mBodies[i]->getBodyCore().getWakeCounter() == 0.0f);
			PX_ASSERT(mBodies[i]->checkSleepReadinessBesidesWakeCounter());
		}
		else
			PX_ASSERT(mBodies[i]->isActive());
	}
#endif
}

PxU32 ev4sio_Sc::ArticulationSim::getRootActorIndex() const
{
	return mBodies[0]->getActorID();
}
