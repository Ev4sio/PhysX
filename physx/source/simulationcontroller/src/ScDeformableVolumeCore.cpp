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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScDeformableSurfaceCore.h"
#include "ScDeformableVolumeCore.h"

#include "ScPhysics.h"
#include "ScDeformableVolumeSim.h"
#include "DyDeformableVolume.h"
#include "GuTetrahedronMesh.h"
#include "GuBV4.h"
#include "geometry/PxTetrahedronMesh.h"


using namespace ev4sio_physx;

ev4sio_Sc::DeformableVolumeCore::DeformableVolumeCore() :
	ActorCore(PxActorType::eDEFORMABLE_VOLUME, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0),
	mGpuMemStat(0)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	// ev4sio_Dy::DeformableCore
	mCore.sleepThreshold = 5e-5f * scale.speed * scale.speed;
	mCore.solverIterationCounts = (1 << 8) | 4;
	mCore.wakeCounter = Physics::sWakeCounterOnCreation;
	mCore.dirty = true;
	
	// ev4sio_Dy::DeformableVolumeCore
	mCore.freezeThreshold = 5e-6f * scale.speed * scale.speed;
}


ev4sio_Sc::DeformableVolumeCore::~DeformableVolumeCore() { }

/////////////////////////////////////////////////////////////////////////////////////////
// PxActor API
/////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::DeformableVolumeCore::setActorFlags(PxActorFlags flags)
{
	mCore.actorFlags = flags;
	mCore.dirty = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableBody API
/////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::DeformableVolumeCore::setBodyFlags(PxDeformableBodyFlags flags)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		const bool wasDisabledSelfCollision = mCore.bodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION;
		const bool isDisabledSelfCollision = flags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION;

		if (wasDisabledSelfCollision != isDisabledSelfCollision)
		{
			if (isDisabledSelfCollision)
				sim->disableSelfCollision();
			else
				sim->enableSelfCollision();
		}
	}

	mCore.bodyFlags = flags;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setLinearDamping(const PxReal v)
{
	mCore.linearDamping = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setMaxLinearVelocity(const PxReal v)
{
	mCore.maxLinearVelocity = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setMaxPenetrationBias(const PxReal v)
{
	mCore.maxPenetrationBias = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setSolverIterationCounts(const PxU16 c)
{
	mCore.solverIterationCounts = c;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setSleepThreshold(const PxReal v)
{
	mCore.sleepThreshold = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setSettlingThreshold(const PxReal v)
{
	mCore.settlingThreshold = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setSettlingDamping(const PxReal v)
{
	mCore.settlingDamping = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setSelfCollisionFilterDistance(const PxReal v)
{
	mCore.selfCollisionFilterDistance = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setWakeCounter(const PxReal v)
{
	setWakeCounterInternal(v);
}

void ev4sio_Sc::DeformableVolumeCore::setWakeCounterInternal(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableVolume API
/////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::DeformableVolumeCore::setVolumeFlags(PxDeformableVolumeFlags flags)
{
	mCore.volumeFlags = flags;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setSelfCollisionStressTolerance(const PxReal v)
{
	mCore.selfCollisionStressTolerance = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::setKinematicTargets(const PxVec4* positions)
{
	mCore.kinematicTarget = positions;
	mCore.dirty = true;
}

PxU32 ev4sio_Sc::DeformableVolumeCore::getGpuIndex() const
{
	const ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	return sim ? sim->getGpuIndex() : 0xffffffff;
}

void ev4sio_Sc::DeformableVolumeCore::addParticleFilter(ev4sio_Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();

	if (sim)
		sim->getScene().addParticleFilter(core, *sim, particleId, userBufferId, tetId);
}

void ev4sio_Sc::DeformableVolumeCore::removeParticleFilter(ev4sio_Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();

	if (sim)
		sim->getScene().removeParticleFilter(core, *sim, particleId, userBufferId, tetId);
}

PxU32 ev4sio_Sc::DeformableVolumeCore::addParticleAttachment(ev4sio_Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addParticleAttachment(core, *sim, particleId, userBufferId, tetId, barycentric);

	return handle;
}

void ev4sio_Sc::DeformableVolumeCore::removeParticleAttachment(ev4sio_Sc::ParticleSystemCore* core, PxU32 handle)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeParticleAttachment(core, *sim, handle);
		setWakeCounterInternal(ScInternalWakeCounterResetValue);
	}
}

void ev4sio_Sc::DeformableVolumeCore::addRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 vertId)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();

	if (sim)
		sim->getScene().addRigidFilter(core, *sim, vertId);

}

void ev4sio_Sc::DeformableVolumeCore::removeRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 vertId)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().removeRigidFilter(core, *sim, vertId);
}

PxU32 ev4sio_Sc::DeformableVolumeCore::addRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 particleId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint, bool doConversion)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if(sim)
		handle = sim->getScene().addRigidAttachment(core, *sim, particleId, actorSpacePose, constraint, doConversion);

	return handle;
}

void ev4sio_Sc::DeformableVolumeCore::removeRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 handle)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeRigidAttachment(core, *sim, handle);
		setWakeCounterInternal(ScInternalWakeCounterResetValue);
	}
}


void ev4sio_Sc::DeformableVolumeCore::addTetRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 tetIdx)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().addTetRigidFilter(core, *sim, tetIdx);
}

void ev4sio_Sc::DeformableVolumeCore::removeTetRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 tetIdx)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeTetRigidFilter(core, *sim, tetIdx);
	}
}

PxU32 ev4sio_Sc::DeformableVolumeCore::addTetRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 tetIdx, const PxVec4& barycentric,
	const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint, bool doConversion)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addTetRigidAttachment(core, *sim, tetIdx, barycentric, actorSpacePose, constraint, doConversion);

	return handle;
}

void ev4sio_Sc::DeformableVolumeCore::addSoftBodyFilter(ev4sio_Sc::DeformableVolumeCore& core, PxU32 tetIdx0, PxU32 tetIdx1)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().addSoftBodyFilter(core, tetIdx0, *sim, tetIdx1);
}

void ev4sio_Sc::DeformableVolumeCore::removeSoftBodyFilter(ev4sio_Sc::DeformableVolumeCore& core, PxU32 tetIdx0, PxU32 tetIdx1)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().removeSoftBodyFilter(core, tetIdx0, *sim, tetIdx1);
}

void ev4sio_Sc::DeformableVolumeCore::addSoftBodyFilters(ev4sio_Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().addSoftBodyFilters(core, *sim, tetIndices0, tetIndices1, tetIndicesSize);
}

void ev4sio_Sc::DeformableVolumeCore::removeSoftBodyFilters(ev4sio_Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().removeSoftBodyFilters(core, *sim, tetIndices0, tetIndices1, tetIndicesSize);
}


PxU32 ev4sio_Sc::DeformableVolumeCore::addSoftBodyAttachment(ev4sio_Sc::DeformableVolumeCore& core, PxU32 tetIdx0, const PxVec4& triBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
	PxConeLimitedConstraint* constraint, PxReal constraintOffset, bool doConversion)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addSoftBodyAttachment(core, tetIdx0, triBarycentric0, *sim, tetIdx1, tetBarycentric1, constraint, constraintOffset, doConversion);

	return handle;
}

void ev4sio_Sc::DeformableVolumeCore::removeSoftBodyAttachment(ev4sio_Sc::DeformableVolumeCore& core, PxU32 handle)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	setWakeCounterInternal(ScInternalWakeCounterResetValue);
	core.setWakeCounterInternal(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeSoftBodyAttachment(core, *sim, handle);
}


void ev4sio_Sc::DeformableVolumeCore::addClothFilter(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 triIdx, PxU32 tetIdx)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();

	if (sim)
		sim->getScene().addClothFilter(core, triIdx, *sim, tetIdx);
}

void ev4sio_Sc::DeformableVolumeCore::removeClothFilter(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 triIdx, PxU32 tetIdx)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
		sim->getScene().removeClothFilter(core, triIdx, *sim, tetIdx);
}

PxU32 ev4sio_Sc::DeformableVolumeCore::addClothAttachment(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric,
	PxConeLimitedConstraint* constraint, PxReal constraintOffset, bool doConversion)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addClothAttachment(core, triIdx, triBarycentric, *sim, tetIdx, tetBarycentric, constraint, constraintOffset, doConversion);

	return handle;
}

void ev4sio_Sc::DeformableVolumeCore::removeClothAttachment(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 handle)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	setWakeCounterInternal(ScInternalWakeCounterResetValue);
	core.setWakeCounterInternal(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeClothAttachment(core, *sim, handle);
}

//---------------------------------------------------------------------------------
// Internal API
//---------------------------------------------------------------------------------

void ev4sio_Sc::DeformableVolumeCore::addMaterial(const PxU16 handle)
{
	mCore.materialHandles.pushBack(handle);
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableVolumeCore::clearMaterials()
{
	mCore.materialHandles.clear();
	mCore.dirty = true;
}

PxActor* ev4sio_Sc::DeformableVolumeCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<DeformableVolumeCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

void ev4sio_Sc::DeformableVolumeCore::attachShapeCore(ShapeCore* shapeCore)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		sim->attachShapeCore(shapeCore);
		mCore.dirty = true;
	}
}

void ev4sio_Sc::DeformableVolumeCore::attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxDeformableVolumeAuxData* simulationState)
{
	ev4sio_Sc::DeformableVolumeSim* sim = getSim();
	if (sim)
	{
		sim->attachSimulationMesh(simulationMesh, simulationState);
		mCore.dirty = true;
	}
}

void ev4sio_Sc::DeformableVolumeCore::onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags)
{
	PX_UNUSED(shape);
	DeformableVolumeSim* sim = getSim();
	if (!sim)
		return;
	ShapeSimBase& s = sim->getShapeSim();

	if (notifyFlags & ShapeChangeNotifyFlag::eGEOMETRY)
		s.onVolumeOrTransformChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eRESET_FILTERING)
		s.onResetFiltering();
	if (notifyFlags & ShapeChangeNotifyFlag::eSHAPE2BODY)
		s.onVolumeOrTransformChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eFILTERDATA)
		s.onFilterDataChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eCONTACTOFFSET)
		s.onContactOffsetChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eRESTOFFSET)
		s.onRestOffsetChange();
}

ev4sio_Sc::DeformableVolumeSim* ev4sio_Sc::DeformableVolumeCore::getSim() const
{
	return static_cast<ev4sio_Sc::DeformableVolumeSim*>(ActorCore::getSim());
}

#endif //PX_SUPPORT_GPU_PHYSX

