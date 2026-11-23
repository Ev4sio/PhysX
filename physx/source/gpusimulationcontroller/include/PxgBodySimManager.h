// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_BODYSIM_MANAGER_H
#define	PXG_BODYSIM_MANAGER_H

#include "foundation/PxBitMap.h"
#include "foundation/PxHashMap.h"
#include "PxNodeIndex.h"
#include "CmPinnableArray.h"
#include "CmIDPool.h"
#include "CmBlockArray.h"
#include "PxsHeapStats.h"
#include "PxgBodySim.h"

namespace ev4sio_physx
{
	class PxsRigidBody;
	struct PxsExternalAccelerationProvider;

	namespace ev4sio_Dy
	{
		class FeatherstoneArticulation;
		class DeformableSurface;
		class DeformableVolume;
		class ParticleSystem;
	}

	struct PxgRemapIndices
	{
	public:
		PxU32 nodeIndex; //this is the index to PxgBodySim in GPU
		PxU32 remapIndex; //this is the index map between PxgBodySim and the PxgArticulation in GPU
	};

	struct PxgArticulationUpdate
	{
		PxU32 articulationIndex; //Which articulation on GPU
		ev4sio_Dy::FeatherstoneArticulation* articulation; //Which articulation on CPU
	};
	struct PxgArticulationIndices	: public PxgRemapIndices {};
	struct PxgSoftBodyIndices		: public PxgRemapIndices {};
	struct PxgFEMClothIndices		: public PxgRemapIndices {};
	struct PxgParticleSystemIndices	: public PxgRemapIndices {};

	struct PxgStaticConstraint
	{
		PxU32 uniqueId;
		PxU32 linkID;
	};

	struct PxgSelfConstraint
	{
		PxU32 uniqueId;
		PxU32 linkID0;
		PxU32 linkID1;
	};

	struct PxgStaticConstraints
	{
		static const PxU32 MaxConstraints = 16;
		PxArray<PxgStaticConstraint> mStaticContacts;
		PxArray<PxgStaticConstraint> mStaticJoints;
		
	};

	struct PxgArticulationSelfConstraints
	{
		static const PxU32 MaxConstraints = 32;
		PxArray<PxgSelfConstraint> mSelfContacts;
		PxArray<PxgSelfConstraint> mSelfJoints;
	};

	class PxgBodySimManager
	{
		PX_NOCOPY(PxgBodySimManager)
	public:
		PxgBodySimManager(ev4sio_Cm::VirtualAllocatorCallback& hostAlloc) :
			mNewUpdatedBodies(hostAlloc, PxsHeapStats::eSIMULATION), 
			mTotalNumBodies(0), mNbUpdatedBodies(0), 
			mTotalNumArticulations(0), mTotalNumSoftBodies(0), mTotalNumFEMCloths(0),
			mTotalNumPBDParticleSystems(0),
			mActivePBDParticleSystems(hostAlloc, PxsHeapStats::eSIMULATION_PARTICLES),
			mActivePBDParticleSystemsDirty(false),
			mActiveSoftbodies(hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
			mActiveSelfCollisionSoftbodies(hostAlloc, PxsHeapStats::eSIMULATION_SOFTBODY),
			mActiveSoftbodiesDirty(false),
			mActiveFEMCloths(hostAlloc, PxsHeapStats::eSIMULATION_FEMCLOTH),
			mActiveFEMClothsDirty(false),
			mTotalStaticArticContacts(0), 
			mTotalStaticArticJoints(0),
			mTotalSelfArticContacts(0),
			mTotalSelfArticJoints(0),
			mMaxStaticArticContacts(0),
			mMaxStaticArticJoints(0),
			mMaxSelfArticContacts(0),
			mMaxSelfArticJoints(0),
			mTotalStaticRBContacts(0),
			mTotalStaticRBJoints(0),
			mMaxStaticRBContacts(0),
			mMaxStaticRBJoints(0),
			mExternalAccelerations(NULL)
		{
		}

		~PxgBodySimManager();

		void	addBody(PxsRigidBody* bodyCore, const PxU32 nodeIndex);

		void	addArticulation(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex, bool OmniPVDRecordDirectGPUAPI);
		void	releaseArticulation(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex);
		void	releaseDeferredArticulationIds();

		void	addSoftBody(ev4sio_Dy::DeformableVolume* deformableVolume, const PxU32 nodeIndex);
		void	releaseSoftBody(ev4sio_Dy::DeformableVolume* deformableVolume);
		void	releaseDeferredSoftBodyIds();
		bool	activateSoftbody(ev4sio_Dy::DeformableVolume* deformableVolume);
		bool	deactivateSoftbody(ev4sio_Dy::DeformableVolume* deformableVolume);

		bool	activateSoftbodySelfCollision(ev4sio_Dy::DeformableVolume* deformableVolume);
		bool	deactivateSoftbodySelfCollision(ev4sio_Dy::DeformableVolume* deformableVolume);

		void	addFEMCloth(ev4sio_Dy::DeformableSurface*, const PxU32 nodeIndex);
		void	releaseFEMCloth(ev4sio_Dy::DeformableSurface*);
		void	releaseDeferredFEMClothIds();
		bool	activateCloth(ev4sio_Dy::DeformableSurface*);
		bool	deactivateCloth(ev4sio_Dy::DeformableSurface*);

		void	addPBDParticleSystem(ev4sio_Dy::ParticleSystem* particleSystem, const PxU32 nodeIndex);
		void	releasePBDParticleSystem(ev4sio_Dy::ParticleSystem* particleSystem);
		void	releaseDeferredPBDParticleSystemIds();

		void	updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies, PxsExternalAccelerationProvider* externalAccelerations);
		void	updateBody(const PxNodeIndex&);
		void	destroy();

		void	updateArticulation(ev4sio_Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex);

		void	reset();
		void	reserve(const PxU32 nbBodies);

		bool	addStaticArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		bool	addStaticArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		bool	addSelfArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);
		bool	removeSelfArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);

		bool	addSelfArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);
		bool	removeSelfArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);

		bool	addStaticRBContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticRBContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		bool	addStaticRBJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticRBJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		PX_INLINE PxU32	getArticulationRemapIndex(PxU32 nodeIndex) { return mNodeToRemapMap.find(nodeIndex)->second;}

		PX_INLINE PxU32 getNumActiveParticleSystem()
		{
			return mActivePBDParticleSystems.size();
		}

		PxArray<PxgArticulationUpdate>							mUpdatedArticulations;
		PxArray<void*>											mBodies; //rigid bodies, articulations, soft bodies and particle systems
		
		PxArray<PxU32>											mNewOrUpdatedBodySims;
		PxArray<PxgArticulationIndices>							mNewArticulationSims;
		PxArray<PxgSoftBodyIndices>								mNewSoftBodySims;
		PxArray<PxgFEMClothIndices>								mNewFEMClothSims;
		PxArray<ev4sio_Dy::DeformableSurface*>							mDeformableSurfaces;
		PxArray<ev4sio_Dy::DeformableVolume*>							mDeformableVolumes;
		PxArray<PxgParticleSystemIndices>						mNewPBDParticleSystemSims;


		ev4sio_Cm::DeferredIDPool										mArticulationIdPool; //generate the remap id between pxgbodysim and pxgarticulation
		ev4sio_Cm::DeferredIDPool										mSoftBodyIdPool; //generate the remap id between pxgbodysim and pxgsoftbody
		ev4sio_Cm::DeferredIDPool										mFEMClothIdPool; //generate the remap id between pxgbodysim and pxgfemcloth
		ev4sio_Cm::DeferredIDPool										mPBDParticleSystemIdPool; //generate the remap id between pxgbodysim and pxgparticlesystem


		ev4sio_Cm::PinnableArray<PxgBodySimVelocityUpdate>				mNewUpdatedBodies;
		PxU32													mTotalNumBodies; //include rigid body and articulation
		PxU32													mNbUpdatedBodies; //this is used for multiply threads in the ScBeforeSolverTask to update body information

		PxBitMap												mUpdatedMap;
		PxU32													mTotalNumArticulations;
		PxU32													mTotalNumSoftBodies;
		PxU32													mTotalNumFEMCloths;
		PxU32													mTotalNumPBDParticleSystems;
		PxArray<PxU32>											mActiveFEMClothIndex;
		PxArray<PxU32>											mActiveSoftbodyIndex;
		PxArray<PxU32>											mActiveSelfCollisionSoftbodyIndex;

		ev4sio_Cm::PinnableArray<PxU32>								mActivePBDParticleSystems;
		bool													mActivePBDParticleSystemsDirty;
		ev4sio_Cm::PinnableArray<PxU32>								mActiveSoftbodies;
		ev4sio_Cm::PinnableArray<PxU32>								mActiveSelfCollisionSoftbodies;
		PxArray<PxU32>											mActiveSoftbodiesStaging;
		PxArray<PxU32>											mActiveSelfCollisionSoftBodiesStaging;
		bool													mActiveSoftbodiesDirty;
		ev4sio_Cm::PinnableArray<PxU32>								mActiveFEMCloths;
		PxArray<PxU32>											mActiveFEMClothStaging;
		bool													mActiveFEMClothsDirty;

		PxHashMap<PxU32, PxU32>									mNodeToRemapMap;
#if PX_SUPPORT_OMNI_PVD
		PxHashMap<PxU32, PxU32>									mRemapToNodeMap;
#endif
		PxBitMap												mPendingArticulationUploads;
		PxArray<PxU32>											mDeferredFreeNodeIDs;

		ev4sio_Cm::BlockArray<PxgStaticConstraints, 1024>				mStaticConstraints;
		ev4sio_Cm::BlockArray<PxgArticulationSelfConstraints, 1024>	mArticulationSelfConstraints;

		PxU32													mTotalStaticArticContacts;
		PxU32													mTotalStaticArticJoints;

		PxU32													mTotalSelfArticContacts;
		PxU32													mTotalSelfArticJoints;

		PxU32													mMaxStaticArticContacts;
		PxU32													mMaxStaticArticJoints;

		PxU32													mMaxSelfArticContacts;
		PxU32													mMaxSelfArticJoints;

		PxU32													mTotalStaticRBContacts;
		PxU32													mTotalStaticRBJoints;

		PxU32													mMaxStaticRBContacts;
		PxU32													mMaxStaticRBJoints;

		PxsExternalAccelerationProvider*						mExternalAccelerations;
	};

}

#endif
