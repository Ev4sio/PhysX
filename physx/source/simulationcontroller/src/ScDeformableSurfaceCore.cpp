// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	mCore.maxLinearVelocity = (v > 1e15f) ? PX_MAX_REAL : v;
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
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().setDynamicsDirty();
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

PxU32 ev4sio_Sc::DeformableSurfaceCore::addRigidAttachments(ev4sio_Sc::BodyCore* core, PxDeformableAttachmentTargetType::Enum feature,
	const PxU32* indices, const PxVec4* barycentrics, const PxTransform& frame, const PxVec4* coords, PxU32 count)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = PX_INVALID_U32;
	if (sim)
		handle = sim->getScene().addSurfaceRigidAttachments(core, *sim, feature, indices, barycentrics, frame, coords, count);

	return handle;
}

void ev4sio_Sc::DeformableSurfaceCore::removeRigidAttachments(ev4sio_Sc::BodyCore* core, PxU32 handle)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeSurfaceRigidAttachments(core, *sim, handle);
		setWakeCounterInternal(ScInternalWakeCounterResetValue);
	}
}

PxU32 ev4sio_Sc::DeformableSurfaceCore::addSurfaceAttachments(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxDeformableAttachmentTargetType::Enum feature0,
	const PxU32* indices0, const PxVec4* barycentrics0, PxDeformableAttachmentTargetType::Enum feature1,
	const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = PX_INVALID_U32;
	if (sim)
		handle = sim->getScene().addSurfaceSurfaceAttachments(*otherCore, feature0, indices0, barycentrics0, *sim,
			feature1, indices1, barycentrics1, count);

	return handle;
}

void ev4sio_Sc::DeformableSurfaceCore::removeSurfaceAttachments(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32 handle)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	setWakeCounterInternal(ScInternalWakeCounterResetValue);
	otherCore->setWakeCounterInternal(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeSurfaceSurfaceAttachments(*otherCore, *sim, handle);
}

void ev4sio_Sc::DeformableSurfaceCore::addRigidFilters(ev4sio_Sc::RigidCore* core, PxU32* triIndices, PxU32 triIndicesSize)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();

	if (sim)
		sim->getScene().addSurfaceRigidFilters(core, *sim, triIndices, triIndicesSize);
}

void ev4sio_Sc::DeformableSurfaceCore::removeRigidFilters(ev4sio_Sc::RigidCore* core, PxU32* triIndices, PxU32 triIndicesSize)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().removeSurfaceRigidFilters(core, *sim, triIndices, triIndicesSize);
}

void ev4sio_Sc::DeformableSurfaceCore::addSurfaceFilters(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32* otherTriIndices, PxU32* triIndices, PxU32 triIndicesSize)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().addSurfaceSurfaceFilters(*otherCore, otherTriIndices, *sim, triIndices, triIndicesSize);
}

void ev4sio_Sc::DeformableSurfaceCore::removeSurfaceFilters(ev4sio_Sc::DeformableSurfaceCore* otherCore, PxU32* otherTriIndices, PxU32* triIndices, PxU32 triIndicesSize)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().removeSurfaceSurfaceFilters(*otherCore, otherTriIndices, *sim, triIndices, triIndicesSize);
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

