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

#include "ScPhysics.h"
#include "ScDeformableSurfaceSim.h"
#include "DyDeformableSurface.h"
#include "GuTetrahedronMesh.h"
#include "GuBV4.h"
#include "geometry/PxTetrahedronMesh.h"
#include "cudamanager/PxCudaContextManager.h"

using namespace ev4sio_physx;

ev4sio_Sc::DeformableSurfaceCore::DeformableSurfaceCore() :
	ActorCore(PxActorType::eDEFORMABLE_SURFACE, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0),
	mGpuMemStat(0)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	// ev4sio_Dy::DeformableCore
	mCore.sleepThreshold = 5e-5f * scale.speed * scale.speed;
	mCore.solverIterationCounts = (1 << 8) | 4;
	mCore.wakeCounter = Physics::sWakeCounterOnCreation;
	mCore.dirty = true;
}

ev4sio_Sc::DeformableSurfaceCore::~DeformableSurfaceCore() { }

/////////////////////////////////////////////////////////////////////////////////////////
// PxActor API
/////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::DeformableSurfaceCore::setActorFlags(PxActorFlags flags)
{
	mCore.actorFlags = flags;
	mCore.dirty = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableBody API
/////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::DeformableSurfaceCore::setBodyFlags(PxDeformableBodyFlags flags)
{
	mCore.bodyFlags = flags;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setLinearDamping(const PxReal v)
{
	mCore.linearDamping = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setMaxLinearVelocity(const PxReal v)
{
	mCore.maxLinearVelocity = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setMaxPenetrationBias(const PxReal v)
{
	mCore.maxPenetrationBias = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setSolverIterationCounts(const PxU16 c)
{
	mCore.solverIterationCounts = c;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setSleepThreshold(const PxReal v)
{
	mCore.sleepThreshold = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setSettlingThreshold(const PxReal v)
{
	mCore.settlingThreshold = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setSettlingDamping(const PxReal v)
{
	mCore.settlingDamping = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setSelfCollisionFilterDistance(const PxReal v)
{
	mCore.selfCollisionFilterDistance = v;
	mCore.dirty = true;
}

//deprecated
void ev4sio_Sc::DeformableSurfaceCore::setSelfCollisionStressTolerance(const PxReal v)
{
	mCore.selfCollisionStressTolerance = v;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setWakeCounter(const PxReal v)
{
	setWakeCounterInternal(v);
}

void ev4sio_Sc::DeformableSurfaceCore::setWakeCounterInternal(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableSurface API
/////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::DeformableSurfaceCore::setSurfaceFlags(PxDeformableSurfaceFlags flags)
{
	mCore.surfaceFlags = flags;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency)
{
	mCore.nbCollisionPairUpdatesPerTimestep = frequency;
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::setNbCollisionSubsteps(const PxU32 frequency)
{
	mCore.nbCollisionSubsteps = frequency;
	mCore.dirty = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Internal API
/////////////////////////////////////////////////////////////////////////////////////////

PxU32 ev4sio_Sc::DeformableSurfaceCore::addRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 particleId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* params)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
	{
		handle = sim->getScene().addRigidAttachment(core, *sim, particleId, actorSpacePose, params);

	}

	return handle;
}

void ev4sio_Sc::DeformableSurfaceCore::removeRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 handle)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeRigidAttachment(core, *sim, handle);
		setWakeCounterInternal(ScInternalWakeCounterResetValue);
	}
}

void ev4sio_Sc::DeformableSurfaceCore::addTriRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 triIdx)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	
	if (sim)
		sim->getScene().addTriRigidFilter(core, *sim, triIdx);

}

void ev4sio_Sc::DeformableSurfaceCore::removeTriRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 triIdx)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().removeTriRigidFilter(core, *sim, triIdx);
}


PxU32 ev4sio_Sc::DeformableSurfaceCore::addTriRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 triIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addTriRigidAttachment(core, *sim, triIdx, barycentric, actorSpacePose, constraint);

	return handle;
}

void ev4sio_Sc::DeformableSurfaceCore::removeTriRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 handle)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeTriRigidAttachment(core, *sim, handle);
		setWakeCounterInternal(ScInternalWakeCounterResetValue);
	}
}

void ev4sio_Sc::DeformableSurfaceCore::addClothFilter(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32 otherTriIdx, PxU32 triIdx)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().addClothFilter(*otherCore, otherTriIdx, *sim, triIdx);
}

void ev4sio_Sc::DeformableSurfaceCore::removeClothFilter(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32 otherTriIdx, PxU32 triIdx)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().removeClothFilter(*otherCore, otherTriIdx, *sim, triIdx);
}

PxU32 ev4sio_Sc::DeformableSurfaceCore::addClothAttachment(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32 otherTriIdx, const PxVec4& otherTriBarycentric, PxU32 triIdx, const PxVec4& triBarycentric)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = 0xFFFFFFFF;
	if (sim)
		handle = sim->getScene().addTriClothAttachment(*otherCore, otherTriIdx, otherTriBarycentric, *sim, triIdx, triBarycentric);

	return handle;
}

void ev4sio_Sc::DeformableSurfaceCore::removeClothAttachment(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32 handle)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	setWakeCounterInternal(ScInternalWakeCounterResetValue);
	otherCore->setWakeCounterInternal(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeTriClothAttachment(*otherCore, *sim, handle);
}

void ev4sio_Sc::DeformableSurfaceCore::addMaterial(const PxU16 handle)
{
	mCore.materialHandles.pushBack(handle);
	mCore.dirty = true;
}

void ev4sio_Sc::DeformableSurfaceCore::clearMaterials()
{
	mCore.materialHandles.clear();
	mCore.dirty = true;
}

PxActor* ev4sio_Sc::DeformableSurfaceCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<DeformableSurfaceCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

void ev4sio_Sc::DeformableSurfaceCore::attachShapeCore(ShapeCore* shapeCore)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->attachShapeCore(shapeCore);
}

void ev4sio_Sc::DeformableSurfaceCore::onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags)
{
	PX_UNUSED(shape);
	DeformableSurfaceSim* sim = getSim();
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

ev4sio_Sc::DeformableSurfaceSim* ev4sio_Sc::DeformableSurfaceCore::getSim() const
{
	return static_cast<ev4sio_Sc::DeformableSurfaceSim*>(ActorCore::getSim());
}

#endif //PX_SUPPORT_GPU_PHYSX

