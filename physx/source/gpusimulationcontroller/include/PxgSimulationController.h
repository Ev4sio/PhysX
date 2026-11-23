// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SIMULATION_CONTROLLER_H
#define	PXG_SIMULATION_CONTROLLER_H

#include "PxgBodySimManager.h"
#include "PxgJointManager.h"
#include "PxgArticulationLink.h"
#include "PxgSoftBody.h"
#include "PxgFEMCloth.h"
#include "PxgAllocatorDesc.h"
#include "PxgAttachmentManager.h"
#include "PxgDeformableFilterManager.h"
#include "PxgSimulationCoreDesc.h"
#include "DyDeformableVolume.h"

#include "PxsSimulationController.h"
#include "CmTask.h"

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

namespace ev4sio_physx
{
	//this is needed to force PhysXSimulationControllerGpu linkage as Static Library!
	void createPxgSimulationController();

	namespace ev4sio_Bp
	{
		class BroadPhase;
	}

	namespace ev4sio_Dy
	{
		struct ArticulationLink;
	}

	class PxsKernelWranglerManager;

	class PxgSimulationCore;
	class PxgParticleSystemCore;
	class PxgPBDParticleSystemCore;
	class PxgSoftBodyCore;
	class PxgFEMClothCore;
	class PxgCudaBuffer;
	class PxgBoundsArray;
	class PxgGpuContext;
	class PxgNphaseImplementationContext;
	class PxgSimulationController;
	class PxgCudaKernelWranglerManager;
	class PxgCudaBroadPhaseSap;

	class PxgCopyToBodySimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mNewBodySimOffset;
		PxU32 mStartIndex;
		PxU32 mNbToProcess;

	public:
		PxgCopyToBodySimTask(PxgSimulationController& controller, PxU32 bodySimOffset, PxU32 startIdx, PxU32 nbToProcess) : ev4sio_Cm::Task(0), mController(controller),
			mNewBodySimOffset(bodySimOffset), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgCopyToBodySimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToBodySimTask)
	};

	class PxgCopyToArticulationSimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mNewBodySimOffset; 
		PxU32 mStartIndex, mNbToProcess;
		PxI32* mSharedArticulationLinksIndex;
		PxI32* mSharedDofIndex;
		PxI32* mSharedSpatialTendonIndex;
		PxI32* mSharedSpatialTendonAttachmentIndex;
		PxI32* mSharedFixedTendonIndex;
		PxI32* mSharedFixedTendonJointIndex;
		PxI32* mSharedArticulationMimicJointIndex;
		PxI32* mSharedPathToRootIndex;

	public:
		PxgCopyToArticulationSimTask(PxgSimulationController& controller, const PxU32 bodySimOffset, PxU32 startIdx, 
			PxU32 nbToProcess, PxI32* sharedArticulationLinksIndex, PxI32* sharedDofIndex, 
			PxI32* sharedSpatialTendonIndex,
			PxI32* sharedSpatialTendonAttachmentsIndex,
			PxI32* sharedFixedTendonIndex,
			PxI32* sharedFixedTendonJointIndex,
			PxI32* sharedArticulationMimicJointIndex,
			PxI32* sharedPathToRootIndex) :
			ev4sio_Cm::Task(0), mController(controller), mNewBodySimOffset(bodySimOffset), mStartIndex(startIdx),
			mNbToProcess(nbToProcess), mSharedArticulationLinksIndex(sharedArticulationLinksIndex),
			mSharedDofIndex(sharedDofIndex), 
			mSharedSpatialTendonIndex(sharedSpatialTendonIndex),
			mSharedSpatialTendonAttachmentIndex(sharedSpatialTendonAttachmentsIndex),
			mSharedFixedTendonIndex(sharedFixedTendonIndex), 
			mSharedFixedTendonJointIndex(sharedFixedTendonJointIndex),
			mSharedArticulationMimicJointIndex(sharedArticulationMimicJointIndex),
			mSharedPathToRootIndex(sharedPathToRootIndex)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgCopyToArticulationSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToArticulationSimTask)
	};

	class PxgUpdateArticulationSimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mStartIndex, mNbToProcess;
		PxI32* mSharedArticulationLinksIndex;
		PxI32* mSharedArticulationDofIndex;
		PxI32* mSharedSpatialTendonIndex;
		PxI32* mSharedSpatialTendonAttachmentIndex;
		PxI32* mSharedFixedTendonIndex;
		PxI32* mSharedFixedTendonJointIndex;
		PxI32* mSharedMimicJointIndex;

	public:
		PxgUpdateArticulationSimTask(PxgSimulationController& controller, PxU32 startIdx,
			PxU32 nbToProcess, PxI32* sharedArticulationLinksLindex, 
			PxI32* sharedArticulationDofIndex,
			PxI32* sharedSpatialTendonIndex,
			PxI32* sharedSpatialTendonAttachmentIndex,
			PxI32* sharedFixedTendonIndex,
			PxI32* sharedFixedTendonJointIndex,
			PxI32* sharedMimicJointIndex) :
			ev4sio_Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess), mSharedArticulationLinksIndex(sharedArticulationLinksLindex),
			mSharedArticulationDofIndex(sharedArticulationDofIndex),
			mSharedSpatialTendonIndex(sharedSpatialTendonIndex),
			mSharedSpatialTendonAttachmentIndex(sharedSpatialTendonAttachmentIndex),
			mSharedFixedTendonIndex(sharedFixedTendonIndex),
			mSharedFixedTendonJointIndex(sharedFixedTendonJointIndex),
			mSharedMimicJointIndex(sharedMimicJointIndex)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgUpdateArticulationSimTask";
		}

	private:
		PX_NOCOPY(PxgUpdateArticulationSimTask)
	};

	class PxgCopyToSoftBodySimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mStartIndex, mNbToProcess;

	public:

		static const PxU32 NbSoftBodiesPerTask = 50;

		PxgCopyToSoftBodySimTask(PxgSimulationController& controller, PxU32 startIdx, PxU32 nbToProcess) :
			ev4sio_Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgCopyToSoftBodySimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToSoftBodySimTask)
	};

	class PxgCopyToFEMClothSimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mStartIndex, mNbToProcess;

	public:

		static const PxU32 NbFEMClothsPerTask = 50;

		PxgCopyToFEMClothSimTask(PxgSimulationController& controller, PxU32 startIdx, PxU32 nbToProcess) :
			ev4sio_Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgCopyToFEMClothSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToFEMClothSimTask)
	};

	class PxgCopyToPBDParticleSystemSimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		PxgParticleSystemCore* core;
		PxU32 mStartIndex, mNbToProcess;

	public:
		PxgCopyToPBDParticleSystemSimTask(PxgSimulationController& controller, PxU32 startIdx, PxU32 nbToProcess) :
			ev4sio_Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgCopyToPBDParticleSystemSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToPBDParticleSystemSimTask)
	};

	class PxgPostCopyToShapeSimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;

	public:
		PxgPostCopyToShapeSimTask(PxgSimulationController& controller) : ev4sio_Cm::Task(0), mController(controller)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgPostCopyToShapeSimTask";
		}
	private:
		PX_NOCOPY(PxgPostCopyToShapeSimTask)
	};

	class PxgPostCopyToBodySimTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mController;
		const bool mEnableBodyAccelerations;

	public:
		PxgPostCopyToBodySimTask(PxgSimulationController& controller, bool enableBodyAccelerations) : ev4sio_Cm::Task(0), mController(controller), mEnableBodyAccelerations(enableBodyAccelerations)
		{
		}

		virtual void runInternal() PX_OVERRIDE;

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgPostCopyToBodySimTask";
		}
	private:
		PX_NOCOPY(PxgPostCopyToBodySimTask)
	};
	class PxgPostUpdateParticleAndSoftBodyTask : public ev4sio_Cm::Task
	{
		PxgSimulationController&	mController;
		PxVec3						mGravity;
		PxReal						mDt;

	public:
		PxgPostUpdateParticleAndSoftBodyTask(PxgSimulationController& controller) : ev4sio_Cm::Task(0), mController(controller) {}

		virtual void runInternal() PX_OVERRIDE;

		void setGravity(const PxVec3 gravity) { mGravity = gravity; }
		void setDt(const PxReal dt) { mDt = dt; }

		virtual const char* getName() const PX_OVERRIDE
		{
			return "PxgPostUpdateParticleAndSoftBodyTask";
		}
	private:
		PX_NOCOPY(PxgPostUpdateParticleAndSoftBodyTask)
	};

	class PxgSimulationController : public PxsSimulationController
	{
		PX_NOCOPY(PxgSimulationController)
	public:
		PxgSimulationController(PxsKernelWranglerManager* gpuWranglerManagers, PxCudaContextManager* cudaContextManager,
			PxgGpuContext* dynamicContext, PxgNphaseImplementationContext* npContext, ev4sio_Bp::BroadPhase* bp, bool useGpuBroadphase,
			PxsSimulationControllerCallback* callback, PxgAllocatorDesc& allocDesc,
			PxU32 maxSoftBodyContacts, PxU32 maxFemClothContacts, PxU32 maxParticleContacts, PxU32 collisionStackSizeBytes, bool enableBodyAccelerations);

		virtual ~PxgSimulationController();

		virtual void addPxgShape(ev4sio_Sc::ShapeSimBase* shapeSimBase, const PxsShapeCore* shapeCore, PxNodeIndex nodeIndex, PxU32 index)	PX_OVERRIDE;
		virtual void setPxgShapeBodyNodeIndex(PxNodeIndex nodeIndex, PxU32 index)	PX_OVERRIDE;
		virtual void removePxgShape(PxU32 index)	PX_OVERRIDE;

		virtual void addDynamic(PxsRigidBody* rigidBody, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void addDynamics(PxsRigidBody** rigidBody, const PxU32* nodeIndex, PxU32 nbBodies)	PX_OVERRIDE;

		virtual void addArticulation(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseArticulation(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseDeferredArticulationIds()	PX_OVERRIDE;


		virtual PxU32 addVolumeRigidAttachments(ev4sio_Dy::DeformableVolume* deformableVolume, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex,
			PxDeformableAttachmentTargetType::Enum feature, const PxU32* indices, const PxVec4* barycentrics,
			const PxTransform& frame, const PxVec4* coords, PxU32 count, const bool isActive)	PX_OVERRIDE;
		virtual void removeVolumeRigidAttachments(ev4sio_Dy::DeformableVolume* deformableVolume, PxU32 handle)	PX_OVERRIDE;

		virtual PxU32 addVolumeVolumeAttachments(ev4sio_Dy::DeformableVolume* deformableVolume0, PxDeformableAttachmentTargetType::Enum feature0,
			const PxU32* indices0, const PxVec4* barycentrics0, ev4sio_Dy::DeformableVolume* deformableVolume1,
			PxDeformableAttachmentTargetType::Enum feature1, const PxU32* indices1, const PxVec4* barycentrics1,
			PxU32 count, const bool addToActive)	PX_OVERRIDE;
		virtual void removeVolumeVolumeAttachments(ev4sio_Dy::DeformableVolume* deformableVolume0, PxU32 handle)	PX_OVERRIDE;

		virtual PxU32 addVolumeSurfaceAttachments(ev4sio_Dy::DeformableVolume* deformableVolume, PxDeformableAttachmentTargetType::Enum feature0,
			const PxU32* indices0, const PxVec4* barycentrics0, ev4sio_Dy::DeformableSurface* deformableSurface,
			PxDeformableAttachmentTargetType::Enum feature1, const PxU32* indices1, const PxVec4* barycentrics1,
			PxU32 count, const bool isActive)	PX_OVERRIDE;
		virtual void removeVolumeSurfaceAttachments(ev4sio_Dy::DeformableVolume* deformableVolume, PxU32 handle)	PX_OVERRIDE;

		virtual void addVolumeRigidFilters(ev4sio_Dy::DeformableVolume* deformableVolume,
			const PxNodeIndex& rigidNodeIndex, PxU32* tetIndices, PxU32 tetIndicesSize)	PX_OVERRIDE;
		virtual void removeVolumeRigidFilters(ev4sio_Dy::DeformableVolume* deformableVolume,
			const PxNodeIndex& rigidNodeIndex, PxU32* tetIndices, PxU32 tetIndicesSize)	PX_OVERRIDE;
		virtual void addVolumeStaticShapeFilters(ev4sio_Dy::DeformableVolume* deformableVolume,
			const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* tetIndices, PxU32 tetIndicesSize)	PX_OVERRIDE;
		virtual void removeVolumeStaticShapeFilters(ev4sio_Dy::DeformableVolume* deformableVolume,
			const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* tetIndices, PxU32 tetIndicesSize)	PX_OVERRIDE;

		virtual void addVolumeVolumeFilters(ev4sio_Dy::DeformableVolume* deformableVolume0, ev4sio_Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1,
			PxU32 tetIndicesSize)	PX_OVERRIDE;
		virtual void removeVolumeVolumeFilters(ev4sio_Dy::DeformableVolume* deformableVolume0, ev4sio_Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1,
			PxU32 tetIndicesSize)	PX_OVERRIDE;

		virtual void addVolumeSurfaceFilters(ev4sio_Dy::DeformableVolume* deformableVolume, ev4sio_Dy::DeformableSurface* deformableSurface, PxU32* triIndices, PxU32* tetIndices,
			PxU32 indicesSize)	PX_OVERRIDE;
		virtual void removeVolumeSurfaceFilters(ev4sio_Dy::DeformableVolume* deformableVolume, ev4sio_Dy::DeformableSurface* deformableSurface, PxU32* triIndices, PxU32* tetIndices,
			PxU32 indicesSize)	PX_OVERRIDE;

		virtual PxU32 addSurfaceRigidAttachments(ev4sio_Dy::DeformableSurface* deformableSurface, PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex,
			PxDeformableAttachmentTargetType::Enum feature, const PxU32* indices, const PxVec4* barycentrics,
			const PxTransform& frame, const PxVec4* coords, PxU32 count, const bool isActive)	PX_OVERRIDE;
		virtual void removeSurfaceRigidAttachments(ev4sio_Dy::DeformableSurface* deformableSurface, PxU32 handle)	PX_OVERRIDE;

		virtual PxU32 addSurfaceSurfaceAttachments(ev4sio_Dy::DeformableSurface* deformableSurface0, PxDeformableAttachmentTargetType::Enum feature0,
			const PxU32* indices0, const PxVec4* barycentrics0, ev4sio_Dy::DeformableSurface* deformableSurface1,
			PxDeformableAttachmentTargetType::Enum feature1, const PxU32* indices1, const PxVec4* barycentrics1,
			PxU32 count, const bool addToActive)	PX_OVERRIDE;
		virtual void removeSurfaceSurfaceAttachments(ev4sio_Dy::DeformableSurface* deformableSurface0, PxU32 handle)	PX_OVERRIDE;

		virtual void addSurfaceRigidFilters(ev4sio_Dy::DeformableSurface* deformableSurface,
			const PxNodeIndex& rigidNodeIndex, PxU32* triIndices, PxU32 triIndicesSize)	PX_OVERRIDE;
		virtual void removeSurfaceRigidFilters(ev4sio_Dy::DeformableSurface* deformableSurface, const PxNodeIndex& rigidNodeIndex, PxU32* triIndices,
			PxU32 triIndicesSize)	PX_OVERRIDE;
		virtual void addSurfaceStaticShapeFilters(ev4sio_Dy::DeformableSurface* deformableSurface,
			const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* triIndices, PxU32 triIndicesSize)	PX_OVERRIDE;
		virtual void removeSurfaceStaticShapeFilters(ev4sio_Dy::DeformableSurface* deformableSurface,
			const PxU32* shapeCacheIds, PxU32 nbShapes, PxU32* triIndices, PxU32 triIndicesSize)	PX_OVERRIDE;

		virtual void addSurfaceSurfaceFilters(ev4sio_Dy::DeformableSurface* deformableSurface0, ev4sio_Dy::DeformableSurface* deformableSurface1, PxU32* triIndices0, PxU32* triIndices1,
			PxU32 triIndicesSize)	PX_OVERRIDE;
		virtual void removeSurfaceSurfaceFilters(ev4sio_Dy::DeformableSurface* deformableSurface0, ev4sio_Dy::DeformableSurface* deformableSurface1, PxU32* triIndices0, PxU32* triIndices1,
			PxU32 triIndicesSize)	PX_OVERRIDE;

		virtual void addSoftBody(ev4sio_Dy::DeformableVolume* deformableVolume, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseSoftBody(ev4sio_Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;

		virtual void releaseDeferredSoftBodyIds()	PX_OVERRIDE;
		virtual PxU32 getNbReservedSoftBodyIds()	const	PX_OVERRIDE;
		virtual void activateSoftbody(ev4sio_Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void deactivateSoftbody(ev4sio_Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void activateSoftbodySelfCollision(ev4sio_Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void deactivateSoftbodySelfCollision(ev4sio_Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void setSoftBodyWakeCounter(ev4sio_Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;

		virtual void addFEMCloth(ev4sio_Dy::DeformableSurface* deformableSurface, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseFEMCloth(ev4sio_Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;
		virtual void releaseDeferredFEMClothIds()	PX_OVERRIDE;
		virtual PxU32 getNbReservedFEMClothIds()	const	PX_OVERRIDE;
		virtual void activateCloth(ev4sio_Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;
		virtual void deactivateCloth(ev4sio_Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;
		virtual void setClothWakeCounter(ev4sio_Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;

		virtual void addParticleSystem(ev4sio_Dy::ParticleSystem* particleSystem, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseParticleSystem(ev4sio_Dy::ParticleSystem* particleSystem)	PX_OVERRIDE;
		virtual void releaseDeferredParticleSystemIds()	PX_OVERRIDE;

		virtual void updateDynamic(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void setKinematicTargetDirectGPU(const PxNodeIndex& nodeIndex, const PxTransform& body2World)	PX_OVERRIDE;
		virtual void updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies, PxsExternalAccelerationProvider* externalAccelerations)	PX_OVERRIDE;
		virtual void addJoint(const ev4sio_Dy::Constraint&) PX_OVERRIDE;
		virtual void updateJoint(const PxU32 edgeIndex, ev4sio_Dy::Constraint* constraint)	PX_OVERRIDE;
		virtual void updateBodies(PxBaseTask* continuation)	PX_OVERRIDE;
		virtual void updateShapes(PxBaseTask* continuation)	PX_OVERRIDE;

		virtual void preIntegrateAndUpdateBound(PxBaseTask* continuation, const PxVec3 gravity, const PxReal dt)	PX_OVERRIDE;
		virtual void updateParticleSystemsAndSoftBodies()	PX_OVERRIDE;
		virtual void sortContacts()	PX_OVERRIDE;
		virtual void update(ev4sio_Cm::PinnableBitMap& changedHandleMap)	PX_OVERRIDE;
		virtual void mergeChangedAABBMgHandle()	PX_OVERRIDE;
		virtual void gpuDmabackData(PxsTransformCache& cache, ev4sio_Bp::BoundsArray& boundArray, ev4sio_Cm::PinnableBitMap& changedAABBMgrHandles, bool enableDirectGPUAPI)	PX_OVERRIDE;
		virtual void updateScBodyAndShapeSim(PxsTransformCache& cache, ev4sio_Bp::BoundsArray& boundArray, PxBaseTask* continuation)	PX_OVERRIDE;
		
		virtual const void* getRigidBodyAccelerations() const PX_OVERRIDE;

		virtual void updateArticulation(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateArticulationJoint(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateArticulationExtAccel(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateArticulationAfterIntegration(PxsContext* /*llContext*/, ev4sio_Bp::AABBManagerBase* /*aabbManager*/,
			PxArray<ev4sio_Sc::BodySim*>& /*ccdBodies*/, PxBaseTask* /*continuation*/, ev4sio_IG::IslandSim& /*islandSim*/, float /*dt*/, bool /*isSleepingDisabled*/) PX_OVERRIDE	{}
		virtual PxU32* getActiveBodies()	PX_OVERRIDE;
		virtual PxU32* getDeactiveBodies()	PX_OVERRIDE;
		virtual void** getRigidBodies()	PX_OVERRIDE;
		virtual PxU32	getNbBodies()	PX_OVERRIDE;

		virtual ev4sio_Sc::ShapeSimBase** getShapeSims()	PX_OVERRIDE;
		virtual PxU32*	getUnfrozenShapes()	PX_OVERRIDE;
		virtual PxU32*	getFrozenShapes()	PX_OVERRIDE;
		virtual PxU32	getNbFrozenShapes()	PX_OVERRIDE;
		virtual PxU32	getNbUnfrozenShapes()	PX_OVERRIDE;
		virtual PxU32	getNbShapes()	PX_OVERRIDE;
		virtual void	clear()	PX_OVERRIDE	{ mNbFrozenShapes = 0; mNbUnfrozenShapes = 0; }
		virtual void	setBounds(ev4sio_Bp::BoundsArray* boundArray)	PX_OVERRIDE;
		virtual void	reserve(const PxU32 nbBodies)	PX_OVERRIDE;

		virtual PX_INLINE PxU32   getArticulationRemapIndex(const PxU32 nodeIndex) PX_OVERRIDE { return mBodySimManager.getArticulationRemapIndex(nodeIndex); }
		
		virtual void	setDeformableSurfaceGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback) PX_OVERRIDE PX_FINAL;
		virtual void	setDeformableVolumeGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback) PX_OVERRIDE PX_FINAL;

		// new direct-GPU API
		virtual bool	getRigidDynamicData(void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const PX_OVERRIDE PX_FINAL;
		virtual bool 	setRigidDynamicData(const void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;
		
		virtual bool 	getArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const PX_OVERRIDE PX_FINAL;
		virtual bool 	setArticulationData(const void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;
		virtual	bool	computeArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;

		virtual bool 	evaluateSDFDistances(PxVec4* localGradientAndSDFConcatenated, const PxShapeGPUIndex* shapeIndices, const PxVec4* localSamplePointsConcatenated, const PxU32* samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;
		virtual	bool	copyContactData(void* data, PxU32* numContactPairs, const PxU32 maxContactPairs, CUevent startEvent, CUevent copyEvent) PX_OVERRIDE PX_FINAL;

		virtual PxArticulationGPUAPIMaxCounts getArticulationGPUAPIMaxCounts()	const	PX_OVERRIDE PX_FINAL;

		virtual bool	getD6JointData(void* data, const PxD6JointGPUIndex* gpuIndices, PxD6JointGPUAPIReadType::Enum dataType, PxU32 nbElements, PxF32 oneOverDt, CUevent startEvent, CUevent finishEvent) const PX_OVERRIDE PX_FINAL;

		void		 initDirectGPUAPIState();
		// end new direct-GPU API

		virtual	PxU32	getInternalShapeIndex(const PxsShapeCore& shapeCore)	PX_OVERRIDE PX_FINAL;

		virtual void	syncParticleData()	PX_OVERRIDE;
		virtual void	updateBoundsAndShapes(ev4sio_Bp::AABBManagerBase& aabbManager, bool useDirectApi)	PX_OVERRIDE;

		PX_FORCE_INLINE PxgSimulationCore* getSimulationCore() { return mSimulationCore; }
		PX_FORCE_INLINE PxgJointManager& getJointManager() { return mJointManager; }
		PX_FORCE_INLINE PxgBodySimManager& getBodySimManager() { return mBodySimManager; }

		PX_FORCE_INLINE PxgPBDParticleSystemCore* getPBDParticleSystemCore() { return mPBDParticleSystemCore; }

		PX_FORCE_INLINE PxgSoftBodyCore* getSoftBodyCore() { return mSoftBodyCore; }

		PX_FORCE_INLINE PxgFEMClothCore* getFEMClothCore() { return mFEMClothCore; }

		PX_FORCE_INLINE PxgSoftBody* getSoftBodies() { return mSoftBodyPool.begin(); }
		PX_FORCE_INLINE PxU32 getNbSoftBodies() { return mSoftBodyPool.size(); }

		PX_FORCE_INLINE PxU32* getActiveSoftBodies() { return mBodySimManager.mActiveSoftbodies.begin(); }
		PX_FORCE_INLINE PxU32 getNbActiveSoftBodies() { return mBodySimManager.mActiveSoftbodies.size(); }

		PX_FORCE_INLINE PxU32* getSoftBodyNodeIndex() { return mSoftBodyNodeIndexPool.begin(); }

		PX_FORCE_INLINE PxgFEMCloth* getFEMCloths() { return mFEMClothPool.begin(); }
		PX_FORCE_INLINE PxU32 getNbFEMCloths() { return mFEMClothPool.size(); }

		PX_FORCE_INLINE PxU32* getActiveFEMCloths() { return mBodySimManager.mActiveFEMCloths.begin(); }
		PX_FORCE_INLINE PxU32 getNbActiveFEMCloths() { return mBodySimManager.mActiveFEMCloths.size(); }

		PX_FORCE_INLINE PxU32* getFEMClothNodeIndex() { return mFEMClothNodeIndexPool.begin(); }

		void postCopyToShapeSim();
		void postCopyToBodySim(bool enableBodyAccelerations);
		//integrate particle system and update bound/update grid/self collision
		void preIntegrateAndUpdateBoundParticleSystem(const PxVec3 gravity, const PxReal dt);
		void preIntegrateAndUpdateBoundSoftBody(const PxVec3 gravity, const PxReal dt);
		void preIntegrateAndUpdateBoundFEMCloth(const PxVec3 gravity, const PxReal dt);
		void updateJointsAndSyncData();

		PX_FORCE_INLINE PxU32 getMaxLinks() { return mMaxLinks; }

		PX_FORCE_INLINE PxU32 getMaxFemContacts() { return mMaxFemClothContacts; }

		virtual PxU32					getNbDeactivatedDeformableSurfaces() const PX_OVERRIDE;
		virtual PxU32					getNbActivatedDeformableSurfaces() const PX_OVERRIDE;

		virtual ev4sio_Dy::DeformableSurface**	getDeactivatedDeformableSurfaces() const PX_OVERRIDE;
		virtual ev4sio_Dy::DeformableSurface**	getActivatedDeformableSurfaces() const PX_OVERRIDE;

		virtual PxU32					getNbDeactivatedDeformableVolumes() const PX_OVERRIDE;
		virtual PxU32					getNbActivatedDeformableVolumes() const PX_OVERRIDE;

		virtual ev4sio_Dy::DeformableVolume**	getDeactivatedDeformableVolumes() const PX_OVERRIDE;
		virtual ev4sio_Dy::DeformableVolume**	getActivatedDeformableVolumes() const PX_OVERRIDE;

		virtual const PxReal*			getDeformableVolumeWakeCounters() const PX_OVERRIDE;


		virtual void setEnableOVDReadback(bool enableOVDReadback) PX_OVERRIDE;
		virtual PxsOVDReadbackMode::Enum getOVDReadbackMode() const PX_OVERRIDE;

		virtual void setEnableOVDCollisionReadback(bool enableOVDCollisionsReadback) PX_OVERRIDE;
		virtual bool getEnableOVDCollisionReadback() const PX_OVERRIDE;
		virtual bool getEnableOVDContactCollection() const PX_OVERRIDE { return mEnableOVDCollisionReadback; }

		virtual bool isArticulationUploadPending(PxU32 gpuIndex) const PX_OVERRIDE { return mBodySimManager.mPendingArticulationUploads.boundedTest(gpuIndex) != 0; }

#if PX_SUPPORT_OMNI_PVD
		virtual void setOVDCallbacks(PxsSimulationControllerOVDCallbacks& ovdCallbacks) PX_OVERRIDE;
		virtual bool ovdReadbackRigidDynamicData(const PxRigidDynamicGPUIndex* indices, PxU32 nbElements, PxRigidDynamicGPUAPIReadType::Enum dataType) PX_OVERRIDE;
		virtual bool ovdReadbackArticulationData(const PxArticulationGPUIndex* indices, PxU32 nbElements, PxArticulationGPUAPIReadType::Enum dataType) PX_OVERRIDE;
		virtual bool ovdIsContactPairCaptured(const PxActor& actor0, const PxActor& actor1) const PX_OVERRIDE;
		virtual bool ovdBeginContactReadback() PX_OVERRIDE;
		virtual void ovdQueueContactReadback(const void* patches, const void* points, const void* friction, const PxReal* forces, PxU32 nbContacts, PxU32 nbPatches) PX_OVERRIDE;
		virtual bool ovdFinishContactReadback() PX_OVERRIDE;
		virtual bool ovdReadbackContactPair(const void* patches, const void* points, const void* friction, const PxReal* forces, PxU32 nbContacts, PxU32 nbPatches) PX_OVERRIDE;
		virtual PxU64 getOmniPvdReadbackBytes() const PX_OVERRIDE { return mOmniPvdReadbackBytes; }
		virtual void recordOmniPvdReadbackBytes(PxU64 bytes) PX_OVERRIDE { mOmniPvdReadbackBytes += bytes; }
		virtual PxU64 getOmniPvdReadbackCopyCount() const PX_OVERRIDE { return mOmniPvdReadbackCopyCount; }
		virtual void recordOmniPvdReadbackCopy(PxU64 bytes) PX_OVERRIDE { mOmniPvdReadbackBytes += bytes; ++mOmniPvdReadbackCopyCount; }
		PX_FORCE_INLINE PxsSimulationControllerOVDCallbacks* getOVDCallbacks() { return mOvdCallbacks; }
		virtual void ovdSnapshotRigidDynamicForces(const PxRigidDynamicGPUIndex* gpuIndices, PxU32 nbElements) PX_OVERRIDE;
		virtual void ovdSnapshotArticulationForces(const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements) PX_OVERRIDE;
#endif
		virtual bool hasDeformableSurfaces() const PX_OVERRIDE	{ return mFEMClothCore != NULL;  }
		virtual bool hasDeformableVolumes() const PX_OVERRIDE	{ return mSoftBodyCore != NULL; }

		bool	 		getRecomputeArticulationBlockFormat() const { return mRecomputeArticulationBlockFormat; }

	private:

		void createDeformableSurfaceCore();
		void createDeformableVolumeCore();

		void copyToGpuBodySim(PxBaseTask* continuation);
		void copyToGpuParticleSystem(PxBaseTask* continuation);
		void copyToGpuSoftBody(PxBaseTask* continuation);
		void copyToGpuFEMCloth(PxBaseTask* continuation);

		void copyToGpuBodySim(const PxU32 bodySimOffset, PxU32 bodyStartIndex, PxU32 nbToCopy);

		void copyToGpuArticulationSim(const PxU32 bodySimOffset, PxU32 startIndex, PxU32 nbToCopy, 
			PxI32* sharedArticulationLinksIndex, PxI32* sharedArticulationDofIndex, 
			PxI32* sharedArticulationSpatialTendonIndex,
			PxI32* sharedArticulationAttachmentIndex,
			PxI32* sharedArticulationFixedTendonIndex,
			PxI32* sharedArticulationTendonJointIndex,
			PxI32* sharedArticulationMimicJointIndex,
			PxI32* sharedArticulationPathToRootIndex);

		void updateGpuArticulationSim(PxU32 startIndex, PxU32 nbToCopy,
			PxI32* sharedArticulationLinksIndex, PxI32* sharedArticulationDofIndex,
			PxI32* sharedSpatialTendonIndex, PxI32* sharedAttachmentIndex,
			PxI32* sharedFixedTendonIndex, PxI32* sharedFixedTendonJointIndex,
			PxI32* sharedMimicJointIndex);

		// Writes the complete staged GPU record of an articulation link at a slot of the link pools: the link,
		// its properties, its poses and its parent. The GPU copies these whole, so every field must be written.
		void stageArticulationLink(PxU32 index, const ev4sio_Dy::ArticulationLink& cpuLink);

		void copyToGpuSoftBodySim(PxU32 startIndex, PxU32 nbToCopy);

		void copyToGpuFEMClothSim(PxU32 startIndex, PxU32 nbToCopy);
		
		void copyToGpuPBDParticleSystemSim(PxU32 startIndex, PxU32 nbToCopy);

		// bounds are shared by NP and BP, so we have the update in the simulation controller.
		// cache is for np, we update it together with bounds due to similar update logic
		void updateBoundsAndTransformCache(ev4sio_Bp::AABBManagerBase& aabbManager, CUstream stream, PxsTransformCache& cache, PxgCudaBuffer& mGpuTransformCache);

		void copyBoundsAndTransforms(ev4sio_Bp::BoundsArray& boundsArray, PxsTransformCache& transformCache,
															  PxgCudaBuffer& gpuTransformCache, PxU32 boundsArraySize,
															  PxU32 totalTransformCacheSize, CUstream npStream);
		void mergeBoundsAndTransformsChanges(PxgBoundsArray& directGPUBoundsArray,
																	  PxsTransformCache& transformCache,
																	  PxgCudaBuffer& gpuTransformCache, PxU32 boundsArraySize, PxU32 totalTransformCacheSize,
																	  PxU32 numChanges, CUstream npStream);

		PxgPostCopyToShapeSimTask								mPostCopyShapeSimTask;
		PxgPostCopyToBodySimTask								mPostCopyBodySimTask;
		PxgPostUpdateParticleAndSoftBodyTask					mPostUpdateParticleSystemTask;
		PxgBodySimManager										mBodySimManager;
		PxgJointManager											mJointManager;

		PxgSimulationCore*										mSimulationCore;
		PxgSoftBodyCore*										mSoftBodyCore;
		PxgFEMClothCore*										mFEMClothCore;
		PxgPBDParticleSystemCore*								mPBDParticleSystemCore;

		PxgGpuContext*											mDynamicContext;
		PxgNphaseImplementationContext*							mNpContext;

		ev4sio_Cm::PinnableArray<PxgBodySim>							mNewBodySimPool;
		ev4sio_Cm::PinnableArray<PxgArticulationLink>					mLinksPool;
		ev4sio_Cm::PinnableArray<PxReal>								mLinkWakeCounterPool;
		ev4sio_Cm::PinnableArray<ev4sio_Cm::UnAlignedSpatialVector>			mLinkAccelPool;
		ev4sio_Cm::PinnableArray<PxgArticulationLinkProp>				mLinkPropPool;
		ev4sio_Cm::PinnableArray<PxgArticulationLinkSleepData>			mLinkSleepDataPool;
		ev4sio_Cm::PinnableArray<ArticulationBitField>					mLinkChildPool;
		ev4sio_Cm::PinnableArray<PxU32>								mLinkParentPool;
		ev4sio_Cm::PinnableArray<PxTransform>							mLinkBody2WorldPool;
		ev4sio_Cm::PinnableArray<PxTransform>							mLinkBody2ActorPool;
		
		ev4sio_Cm::PinnableArray<ev4sio_Dy::ArticulationJointCore>				mJointPool;
		ev4sio_Cm::PinnableArray<ev4sio_Dy::ArticulationJointCoreData>			mJointDataPool;
		ev4sio_Cm::PinnableArray<PxgArticulationSimUpdate>					mLinkJointIndexPool; //this record the start index of the link for an articulation in an array
		ev4sio_Cm::PinnableArray<PxgArticulation>							mArticulationPool;
		ev4sio_Cm::PinnableArray<PxGpuSpatialTendonData>					mSpatialTendonParamPool;
		ev4sio_Cm::PinnableArray<PxgArticulationTendon>					mSpatialTendonPool;
		ev4sio_Cm::PinnableArray<PxgArticulationTendonElementFixedData>	mAttachmentFixedPool;
		ev4sio_Cm::PinnableArray<PxGpuTendonAttachmentData>				mAttachmentModPool;
		ev4sio_Cm::PinnableArray<PxU32>									mTendonAttachmentMapPool; //store each start index of the attachment to the corresponding tendons
		ev4sio_Cm::PinnableArray<PxGpuFixedTendonData>						mFixedTendonParamPool;
		ev4sio_Cm::PinnableArray<PxgArticulationTendon>					mFixedTendonPool;
		ev4sio_Cm::PinnableArray<PxgArticulationTendonElementFixedData>	mTendonJointFixedDataPool;
		ev4sio_Cm::PinnableArray<PxGpuTendonJointCoefficientData>			mTendonJointCoefficientDataPool;
		ev4sio_Cm::PinnableArray<PxU32>									mTendonTendonJointMapPool; //store each start index of the attachment to the corresponding tendons

		ev4sio_Cm::PinnableArray<PxU32>									mPathToRootPool;

		ev4sio_Cm::PinnableArray<ev4sio_Dy::ArticulationMimicJointCore>			mMimicJointPool;

		ev4sio_Cm::PinnableArray<PxgArticulationSimUpdate>					mArticulationUpdatePoolMapped;	//Articulation update headers
		ev4sio_Cm::PinnableArray<PxReal>									mArticulationDofDataPoolMapped;	//Articulation dof information (jointV, jointP etc.)
		ev4sio_Cm::PinnableArray<PxgSoftBody>								mNewSoftBodyPool;
		PxArray<PxgSoftBodyData>									mNewSoftBodyDataPool;
		ev4sio_Cm::PinnableArray<PxgSoftBody>								mSoftBodyPool;
		PxArray<PxgSoftBodyData>									mSoftBodyDataPool;
		ev4sio_Cm::PinnableArray<PxU32>									mSoftBodyElementIndexPool;
		PxArray<PxU32>												mNewSoftBodyNodeIndexPool;
		PxArray<PxU32>												mNewSoftBodyElementIndexPool;
		PxArray<PxU32>												mSoftBodyNodeIndexPool;
		PxArray<PxU32>												mNewTetMeshByteSizePool;

		PxgAttachmentManager										mVolumeRigidAttachments;
		PxgAttachmentManager										mVolumeVolumeAttachments;
		PxgAttachmentManager										mVolumeSurfaceAttachments;

		PxgDeformableFilterManager								mVolumeRigidFilters;
		PxgDeformableFilterManager								mVolumeVolumeFilters;
		PxgDeformableFilterManager								mVolumeSurfaceFilters;

		ev4sio_Cm::PinnableArray<PxgFEMCloth>								mNewFEMClothPool;
		PxArray<PxgFEMClothData>									mNewFEMClothDataPool;
		ev4sio_Cm::PinnableArray<PxgFEMCloth>								mFEMClothPool;
		PxArray<PxgFEMClothData>									mFEMClothDataPool;
		ev4sio_Cm::PinnableArray<PxU32>									mFEMClothElementIndexPool;
		PxArray<PxU32>												mNewFEMClothNodeIndexPool;
		PxArray<PxU32>												mNewFEMClothElementIndexPool;
		PxArray<PxU32>												mFEMClothNodeIndexPool;
		PxArray<PxU32>												mNewTriangleMeshByteSizePool;

		// Mesh topology per deformable surface, keyed by its GPU remap id. Derived when the surface joins
		// a scene, which is before its element filters register, and dropped when it leaves.
		PxArray<PxgDeformableSurfaceTopology*>						mSurfaceTopologies;

		PxgAttachmentManager										mSurfaceRigidAttachments;
		PxgAttachmentManager										mSurfaceSurfaceAttachments;

		PxgDeformableFilterManager								mSurfaceRigidFilters;
		PxgDeformableFilterManager								mSurfaceSurfaceFilters;

		ev4sio_Cm::PinnableArray<PxU32>									mFrozenPool;
		ev4sio_Cm::PinnableArray<PxU32>									mUnfrozenPool;
		ev4sio_Cm::PinnableArray<PxU32>									mActivatePool;
		ev4sio_Cm::PinnableArray<PxU32>									mDeactivatePool;

		PxU32													mNbFrozenShapes;
		PxU32													mNbUnfrozenShapes;
		bool													mHasBeenSimulated;//if there are no bodies in the scene, we don't run the update method so that we shouldn't need to syncback data

		PxI32													mSharedLinkIndex;
		PxI32													mSharedDofIndex;
		PxI32													mSharedSpatialTendonIndex;
		PxI32													mSharedSpatialAttachmentIndex;
		PxI32													mSharedFixedTendonIndex;
		PxI32													mSharedFixedTendonJointIndex;
		PxI32													mSharedMimicJointIndex;
		PxI32													mSharedPathToRootIndex;

		PxgCudaKernelWranglerManager*							mGpuWranglerManager;
		PxCudaContextManager*									mCudaContextManager;
		PxgAllocatorDesc										mAllocDesc;
		PxgCudaBroadPhaseSap*									mBroadPhase;
		PxU32													mMaxLinks;
		PxU32													mMaxDofs;
		PxU32													mMaxMimicJoints;
		PxU32													mMaxSpatialTendons;
		PxU32													mMaxAttachments;
		PxU32													mMaxFixedTendons;
		PxU32													mMaxTendonJoints;
		PxU32													mMaxPathToRoots;

		PxU32													mMaxSoftBodyContacts;
		PxU32													mMaxFemClothContacts;
		PxU32													mMaxParticleContacts;
		PxU32													mCollisionStackSizeBytes;

		bool 													mRecomputeArticulationBlockFormat;
		bool													mEnableOVDReadback;
		bool													mEnableOVDCollisionReadback;
#if PX_SUPPORT_OMNI_PVD
		PxsSimulationControllerOVDCallbacks*					mOvdCallbacks;
		PxU64 mOmniPvdReadbackBytes;
		PxU64 mOmniPvdReadbackCopyCount;
		struct OmniPvdContactPair
		{
			const void* patches;
			const void* points;
			const void* friction;
			const PxReal* forces;
			PxU32 nbContacts;
			PxU32 nbPatches;
		};
		struct OmniPvdContactRange { PxU64 offset; PxU64 bytes; };
		struct OmniPvdContactRangeLess
		{
			bool operator()(const OmniPvdContactRange& first, const OmniPvdContactRange& second) const { return first.offset < second.offset; }
		};
		PxArray<OmniPvdContactPair> mOmniPvdContactPairs;
		PxArray<OmniPvdContactRange> mOmniPvdContactRanges;
		bool mOmniPvdContactBatchPrepared;
#endif
		friend class PxgCopyToBodySimTask;
		friend class PxgCopyToArticulationSimTask;
		friend class PxgUpdateArticulationSimTask;
		friend class PxgCopyToSoftBodySimTask;
		friend class PxgCopyToFEMClothSimTask;
		friend class PxgCopyToPBDParticleSystemSimTask;
		friend class PxgSimulationCore;
	};

}

#endif
