// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScDeformableVolumeSim.h"
#include "geometry/PxTetrahedronMesh.h"

using namespace ev4sio_physx;
using namespace ev4sio_Dy;

ev4sio_Sc::DeformableVolumeSim::DeformableVolumeSim(DeformableVolumeCore& core, Scene& scene) :
	GPUActorSim(scene, core, NULL)
{
	mLLDeformableVolume = scene.createLLDeformableVolume(this);

	mNodeIndex = scene.getSimpleIslandManager()->addNode(false, false, ev4sio_IG::Node::eDEFORMABLE_VOLUME_TYPE, mLLDeformableVolume);

	scene.getSimpleIslandManager()->activateNode(mNodeIndex);

	mLLDeformableVolume->setElementId(mShapeSim.getElementID());
}

ev4sio_Sc::DeformableVolumeSim::~DeformableVolumeSim()
{
	if (!mLLDeformableVolume)
		return;

	mScene.destroyLLDeformableVolume(*mLLDeformableVolume);

	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);

	mCore.setSim(NULL);
}

bool ev4sio_Sc::DeformableVolumeSim::isSleeping() const
{
	ev4sio_IG::IslandSim& sim = mScene.getSimpleIslandManager()->getAccurateIslandSim();
	return sim.getActiveNodeIndex(mNodeIndex) == PX_INVALID_NODE;
}

void ev4sio_Sc::DeformableVolumeSim::onSetWakeCounter()
{
	mScene.getSimulationController()->setSoftBodyWakeCounter(mLLDeformableVolume);
	if (mLLDeformableVolume->getCore().wakeCounter > 0.f)
		mScene.getSimpleIslandManager()->activateNode(mNodeIndex);
	else
		mScene.getSimpleIslandManager()->deactivateNode(mNodeIndex);
}

void ev4sio_Sc::DeformableVolumeSim::attachShapeCore(ShapeCore* core)
{
	mShapeSim.setCore(core);
	{
		PX_ASSERT(getWorldBounds().isFinite());

		const PxU32 index = mShapeSim.getElementID();

		mScene.getBoundsArray().setBounds(getWorldBounds(), index);

		addToAABBMgr(ev4sio_Bp::FilterType::DEFORMABLE_VOLUME);
	}

	mLLDeformableVolume->setShapeCore(core);
}

PxBounds3 ev4sio_Sc::DeformableVolumeSim::getWorldBounds() const
{
	const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(mShapeSim.getCore().getGeometry());

	return tetGeom.tetrahedronMesh->getLocalBounds();
}

void ev4sio_Sc::DeformableVolumeSim::attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxDeformableVolumeAuxData* simulationState)
{
	mLLDeformableVolume->setSimShapeCore(simulationMesh, simulationState);
}

PxTetrahedronMesh* ev4sio_Sc::DeformableVolumeSim::getSimulationMesh()
{
	return mLLDeformableVolume->getSimulationMesh();
}

PxDeformableVolumeAuxData* ev4sio_Sc::DeformableVolumeSim::getAuxData()
{
	return mLLDeformableVolume->getAuxData();
}

PxTetrahedronMesh* ev4sio_Sc::DeformableVolumeSim::getCollisionMesh()
{
	return mLLDeformableVolume->getCollisionMesh();
}

void ev4sio_Sc::DeformableVolumeSim::enableSelfCollision()
{
	if (isActive())
	{
		mScene.getSimulationController()->activateSoftbodySelfCollision(mLLDeformableVolume);
	}
}

void ev4sio_Sc::DeformableVolumeSim::disableSelfCollision()
{
	if (isActive())
	{
		mScene.getSimulationController()->deactivateSoftbodySelfCollision(mLLDeformableVolume);
	}
}

/*void ev4sio_Sc::DeformableVolumeSim::activate()
{
	// Activate body
	//{
	//	PX_ASSERT((!isKinematic()) || notInScene() || readInternalFlag(InternalFlags(BF_KINEMATIC_MOVED | BF_KINEMATIC_SURFACE_VELOCITY)));	// kinematics should only get activated when a target is set.
	//																																		// exception: object gets newly added, then the state change will happen later
	//	if (!isArticulationLink())
	//	{
	//		mLLBody.mInternalFlags &= (~PxsRigidBody::eFROZEN);
	//		// Put in list of activated bodies. The list gets cleared at the end of a sim step after the sleep callbacks have been fired.
	//		getScene().onBodyWakeUp(this);
	//	}

	//	BodyCore& core = getBodyCore();
	//	if (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
	//	{
	//		PX_ASSERT(!getScene().isInPosePreviewList(*this));
	//		getScene().addToPosePreviewList(*this);
	//	}
	//	createSqBounds();
	//}

	activateInteractions(*this);
}

void ev4sio_Sc::DeformableVolumeSim::deactivate()
{
	deactivateInteractions(*this);

	// Deactivate body
	//{
	//	PX_ASSERT((!isKinematic()) || notInScene() || !readInternalFlag(BF_KINEMATIC_MOVED));	// kinematics should only get deactivated when no target is set.
	//																							// exception: object gets newly added, then the state change will happen later
	//	BodyCore& core = getBodyCore();
	//	if (!readInternalFlag(BF_ON_DEATHROW))
	//	{
	//		// Set velocity to 0.
	//		// Note: this is also fine if the method gets called because the user puts something to sleep (this behavior is documented in the API)
	//		PX_ASSERT(core.getWakeCounter() == 0.0f);
	//		const PxVec3 zero(0.0f);
	//		core.setLinearVelocityInternal(zero);
	//		core.setAngularVelocityInternal(zero);

	//		setForcesToDefaults(!(mLLBody.mInternalFlags & PxsRigidBody::eDISABLE_GRAVITY));
	//	}

	//	if (!isArticulationLink())  // Articulations have their own sleep logic.
	//		getScene().onBodySleep(this);

	//	if (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
	//	{
	//		PX_ASSERT(getScene().isInPosePreviewList(*this));
	//		getScene().removeFromPosePreviewList(*this);
	//	}
	//	destroySqBounds();
	//}
}*/

PxU32 ev4sio_Sc::DeformableVolumeSim::getGpuIndex() const
{
	return mLLDeformableVolume->getGpuIndex();
}

#endif //PX_SUPPORT_GPU_PHYSX
