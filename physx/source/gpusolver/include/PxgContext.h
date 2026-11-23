// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_CONTEXT_H
#define PXG_CONTEXT_H

#include "DyContext.h"
#include "PxgConstraintPartition.h"
#include "PxgSolverBody.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgConstraintPrep.h"
#include "PxvNphaseImplementationContext.h"

namespace ev4sio_physx
{
	class PxgCudaBroadPhaseSap;
	class PxgSolverCore;
	class PxgGpuNarrowphaseCore;
	class PxgArticulationCore;
	class PxgSimulationCore;
	class PxgSoftBodyCore;
	class PxgFEMClothCore;
	class PxgSimulationController;
	struct PxgIslandContext;
	struct PxgAllocatorDesc;
	struct PxsTorsionalFrictionData;
	class PxgParticleSystemCore;

	// PT: TODO: all these tasks are missing a proper context ID for the profiler...

	class PxgCpuJointPrePrepTask : public ev4sio_Cm::Task
	{
		PxgSimulationController& mSimController;

		const ev4sio_Dy::Constraint*const* mConstraints;
		PxgConstraintData* mConstraintData;
		Px1DConstraint* mConstraintRows;

		const PxU32 mStartIndex;
		const PxU32 mNbToProcess;
		const PxU32 mGpuJointOffset;

		PxI32* mRowCounts;

		PX_NOCOPY(PxgCpuJointPrePrepTask)

	public:
		PxgCpuJointPrePrepTask(PxgSimulationController& simConstroller, PxU32 startIndex, PxU32 nbToProcess, PxU32 gpuJointOffset,
			const ev4sio_Dy::Constraint*const* constraints, PxgConstraintData* constraintData, Px1DConstraint* constraintRows, PxI32* rowCounts) :
			ev4sio_Cm::Task(0), mSimController(simConstroller), mConstraints(constraints), mConstraintData(constraintData), mConstraintRows(constraintRows),
			mStartIndex(startIndex), mNbToProcess(nbToProcess), mGpuJointOffset(gpuJointOffset), mRowCounts(rowCounts)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgTGSCpuJointPrePrepTask";
		}
	};

	class PxgGpuContext;

	class PxgCpuPreIntegrationTask : public ev4sio_Cm::Task
	{
		PxgGpuContext& mContext;

		PX_NOCOPY(PxgCpuPreIntegrationTask)

	public:

		PxgCpuPreIntegrationTask(PxgGpuContext& context) : ev4sio_Cm::Task(0), mContext(context)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuPreIntegrationTask";
		}
	};

	class PxgCpuContactPrePrepTask : public ev4sio_Cm::Task
	{
		//From the below, we should be able to iterate over the partitions, process contact pairs
		const PxgIncrementalPartition& mPartition;
		const PxU32 mPartitionIndex;
		const PxU32 mStartIndexWithinPartition;
		const PxU32 mNbToProcess;

		const PxU32* mStartSlabIter;
		const PxU32 mStartSlabOffset;
		const PxU32* mContactStartIndices;

		PxgConstraintBatchHeader* mBatchHeaders;
		const PxU32 mNumBatches;
		const PxU32 mWorkUnitStartIndex;

		PxU32* mPinnedEdgeIds;

		const PxsContactManagerOutputIterator& mOutputIterator;

		const PxU8* mBaseContactPatch;
		const PxU8* mBaseContactPointer;

		PX_NOCOPY(PxgCpuContactPrePrepTask)

	public:
		PxgCpuContactPrePrepTask(const PxgIncrementalPartition& partition, PxU32 partitionIndex, PxU32 startIndexWithinPartition, PxU32 nbToProcess,
			const PxU32* startSlabIter, PxU32 startSlabOffset, const PxU32* contactStartIndices,
			PxgConstraintBatchHeader* batchHeaders, PxU32 nbBatches, PxU32 workUnitStartIndex,
			PxU32* pinnedEdgeIds, PxsContactManagerOutputIterator& outputIter,
			const PxU8* baseContactPatch, const PxU8* baseContactPointer) : ev4sio_Cm::Task(0),
			mPartition(partition), mPartitionIndex(partitionIndex), mStartIndexWithinPartition(startIndexWithinPartition), mNbToProcess(nbToProcess),
			mStartSlabIter(startSlabIter), mStartSlabOffset(startSlabOffset), mContactStartIndices(contactStartIndices),
			mBatchHeaders(batchHeaders), mNumBatches(nbBatches), mWorkUnitStartIndex(workUnitStartIndex),
			mPinnedEdgeIds(pinnedEdgeIds), mOutputIterator(outputIter),
			mBaseContactPatch(baseContactPatch), mBaseContactPointer(baseContactPointer)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuContactPrePrepTask";
		}
	};

	class PxgCpuConstraintPrePrepTask : public ev4sio_Cm::Task
	{
		const PartitionIndices& mEdgeIds;
		const PxU32 mStartEdgeIdx;
		const PxU32 mNumEdges;
		PxgConstraintBatchHeader* mBatchHeaders;

		const PxU32 mNumBatches;
		const PxU32 mConstraintBlockStartIndex;
		const PxU32 mUniqueIdStartIndex;

		PxU32* mPinnedEdgeIds;

		const PxgConstraintPrePrep* mConstraintPrePrep;

		PX_NOCOPY(PxgCpuConstraintPrePrepTask)

	public:

		static const PxU32 NbConstraintsPerTaskTGS = 2048u;
		static const PxU32 NbConstraintsPerTaskPGS = 8192u;

		PxgCpuConstraintPrePrepTask(const PartitionIndices& edgeIds, PxU32 startEdgeIdx, PxU32 nbEdges, PxgConstraintBatchHeader* batchHeaders, PxU32 nbBatches,
			PxU32 constraintBlockStartIndex, PxU32 uniqueIdStartIndex, PxU32* pinnedEdgeIds,
			const PxgConstraintPrePrep* constraintPrePrep) : ev4sio_Cm::Task(0),
			mEdgeIds(edgeIds), mStartEdgeIdx(startEdgeIdx), mNumEdges(nbEdges), mBatchHeaders(batchHeaders), mNumBatches(nbBatches),
			mConstraintBlockStartIndex(constraintBlockStartIndex), mUniqueIdStartIndex(uniqueIdStartIndex), mPinnedEdgeIds(pinnedEdgeIds)
			, mConstraintPrePrep(constraintPrePrep)
		{
		}

		virtual void runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuConstraintPrePrepTask";
		}
	};

	//this include contact and constraint
	class PxgCpuArtiConstraintPrePrepTask : public ev4sio_Cm::Task
	{
		const PartitionIndices&		mEdgeIds;
		const PxU32					mStartEdgeIdx;
		const PxU32					mNumEdges;
		PxgConstraintBatchHeader*	mBatchHeaders;

		const PxU32					mNumBatches;
		const PxU32					mConstraintBlockStartIndex;
		const PxU32					mUniqueIdStartIndex;

		PxU32*						mPinnedEdgeIds;

		const PxgConstraintPrePrep*	mConstraintPrePrep;
		const bool					mIsContact;

		PX_NOCOPY(PxgCpuArtiConstraintPrePrepTask)

	public:

		static const PxU32 NbConstraintsPerTaskPGS = 8192u;
		static const PxU32 NbConstraintsPerTaskTGS = 512u;

		PxgCpuArtiConstraintPrePrepTask(const PartitionIndices& edgeIds, PxU32 startEdgeIdx, PxU32 nbEdges, PxgConstraintBatchHeader* batchHeaders,
			PxU32 nbBatches, PxU32 constraintBlockStartIndex, PxU32 uniqueIdStartIndex, PxU32* pinnedEdgeIds,
			const PxgConstraintPrePrep* constraintPrePrep, bool isContact) : ev4sio_Cm::Task(0),
			mEdgeIds(edgeIds), mStartEdgeIdx(startEdgeIdx), mNumEdges(nbEdges), mBatchHeaders(batchHeaders), mNumBatches(nbBatches),
			mConstraintBlockStartIndex(constraintBlockStartIndex), mUniqueIdStartIndex(uniqueIdStartIndex), mPinnedEdgeIds(pinnedEdgeIds)
			, mConstraintPrePrep(constraintPrePrep), mIsContact(isContact)
		{
		}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;

		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuArtiConstraintPrePrepTask";
		}
	};

	class PxgCpuPrepTask : public ev4sio_Cm::Task
	{
		PxgGpuContext& mContext;
		
		PX_NOCOPY(PxgCpuPrepTask)

	public:
							PxgCpuPrepTask(PxgGpuContext& context) : ev4sio_Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgCpuPrepTask";
		}
	};

	class PxgGpuPrePrepTask : public ev4sio_Cm::Task
	{
		PxgGpuContext& mContext;

		PX_NOCOPY(PxgGpuPrePrepTask)

	public:
							PxgGpuPrePrepTask(PxgGpuContext& context) : ev4sio_Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char* getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgGpuPrePrepTask";
		}
	};

	class PxgPostSolveTask : public ev4sio_Cm::Task
	{
		PxgGpuContext& mContext;
	
		PX_NOCOPY(PxgPostSolveTask)

	public:
							PxgPostSolveTask(PxgGpuContext& context) : ev4sio_Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgPostSolveTask";
		}
	};

	class PxgGpuTask : public ev4sio_Cm::Task
	{
		PxgGpuContext&		mContext;
		PxU32				mMaxNodes;
		ev4sio_Cm::PinnableBitMap*	mChangedHandleMap;//this is for the simulation controller
		
		PX_NOCOPY(PxgGpuTask)

	public:
							PxgGpuTask(PxgGpuContext& context) : ev4sio_Cm::Task(0), mContext(context), mMaxNodes(0), mChangedHandleMap(NULL)	{}

				void		setMaxNodesAndWordCounts(const PxU32 maxNodes, ev4sio_Cm::PinnableBitMap& changedHandleMap) { mMaxNodes = maxNodes; mChangedHandleMap = &changedHandleMap; }

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgGpuTask";
		}
	};

	class PxgGpuIntegrationTask : public ev4sio_Cm::Task
	{
		PxgGpuContext& mContext;

		PX_NOCOPY(PxgGpuIntegrationTask)
	public:
							PxgGpuIntegrationTask(PxgGpuContext& context) : ev4sio_Cm::Task(0), mContext(context)	{}

		virtual void		runInternal() PX_OVERRIDE PX_FINAL;
		virtual const char*	getName() const PX_OVERRIDE PX_FINAL
		{
			return "PxgGpuIntegrationTask";
		}
	};

	class PxgGpuContext : public ev4sio_Dy::Context
	{
		PX_NOCOPY(PxgGpuContext)

	public:

		PxgGpuContext(ev4sio_Cm::FlushPool& flushPool, ev4sio_IG::SimpleIslandManager& islandManager,
			PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions,
			PxReal maxBiasCoefficient, PxvSimStats& simStats, PxgAllocatorDesc& allocDesc,
			PxReal lengthScale, PxU64 contextID, bool isTGS, PxSceneFlags sceneFlags);

		virtual ~PxgGpuContext();

		PX_FORCE_INLINE PxgSolverCore* getGpuSolverCore() { return mGpuSolverCore;}

		PX_FORCE_INLINE PxgArticulationCore* getArticulationCore() { return mGpuArticulationCore; }

		PX_FORCE_INLINE PxgGpuNarrowphaseCore* getNarrowphaseCore() { return mGpuNpCore; }

#if PX_SUPPORT_OMNI_PVD
		struct OmniPvdContactStreams
		{
			PxcDataStreamPool points, patches, forces, friction;
		};
		PX_FORCE_INLINE const OmniPvdContactStreams& getOmniPvdContactStreams() const { return mOvdContactStreams; }
#endif

		PX_FORCE_INLINE PxgSimulationCore* getSimulationCore() { return mGpuSimulationCore; }

		PX_FORCE_INLINE PxgCudaBroadPhaseSap*	getGpuBroadPhase() { return mGpuBp;  }

		PX_FORCE_INLINE PxgSoftBodyCore*	getGpuSoftBodyCore() { return mGpuSoftBodyCore; }

		PX_FORCE_INLINE PxgFEMClothCore*	getGpuFEMClothCore() { return mGpuFEMClothCore; }

		PX_FORCE_INLINE PxgParticleSystemCore**	getGpuParticleSystemCores() { return mGpuParticleSystemCores.begin(); }
		PX_FORCE_INLINE PxU32	getNbGpuParticleSystemCores() { return mGpuParticleSystemCores.size(); }

		PxgParticleSystemCore* getGpuParticleSystemCore();

		PX_FORCE_INLINE PxU32 getCurrentContactStreamIndex() { return mCurrentContactStream; }

		PX_FORCE_INLINE ev4sio_Cm::FlushPool&	getFlushPool() { return mFlushPool; }

		PX_FORCE_INLINE PxU8* getPatchStream(const PxU32 index) { return mPatchStreamAllocators[index]->mStart; }
		PX_FORCE_INLINE PxU8* getContactStream(const PxU32 index) { return mContactStreamAllocators[index]->mStart; }

		PX_FORCE_INLINE bool enforceConstraintWriteBackToHostCopy() const { return mEnforceConstraintWriteBackToHostCopy; }

		//this method make sure we get PxgSimultionController instead of PxsSimulationController
		PxgSimulationController*			getSimulationController();

		virtual void						setSimulationController(PxsSimulationController* mSimulationController)	PX_OVERRIDE;

		virtual void						mergeResults()	PX_OVERRIDE;
		virtual void						getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndexStreamBase)	PX_OVERRIDE;
		
		virtual void						updateBodyCore(PxBaseTask* continuation)	PX_OVERRIDE;

		virtual void						update(	ev4sio_Cm::FlushPool& flushPool, PxBaseTask* continuation, PxBaseTask* postPartitioningTask, PxBaseTask* lostTouchTask,
													PxvNphaseImplementationContext* nphase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt,
													const PxVec3& gravity, ev4sio_Cm::PinnableBitMap& changedHandleMap)	PX_OVERRIDE;

		virtual void						updatePostPartitioning(	PxBaseTask* lostTouchTask,
																	PxvNphaseImplementationContext* nphase, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt,
																	const PxVec3& gravity, ev4sio_Cm::PinnableBitMap& changedHandleMap)	PX_OVERRIDE;

		virtual void setActiveBreakableConstraintCount(PxU32 activeBreakableConstraintCount) PX_OVERRIDE
		{
			mEnforceConstraintWriteBackToHostCopy = (activeBreakableConstraintCount > 0);
		}

		//this is the pre-prepare code for block format joints loaded from the non-block format joints
		void								doConstraintJointBlockPrePrepGPU();

		void								doStaticArticulationConstraintPrePrep(ev4sio_physx::PxBaseTask* continuation, const PxU32 articulationConstraintBatchIndex, const PxU32 articulationContactBatchIndex);

		void								doStaticRigidConstraintPrePrep(ev4sio_physx::PxBaseTask* continuation);
		
		void								doConstraintSolveGPU(PxU32 maxNodes, ev4sio_Cm::PinnableBitMap& changedHandleMap);

		void								doPostSolveTask(ev4sio_physx::PxBaseTask* continuation);

		virtual void						processPatches(	ev4sio_Cm::FlushPool& flushPool, PxBaseTask* continuation,
															PxsContactManager** lostFoundPatchManagers, PxU32 nbLostFoundPatchManagers, PxsContactManagerOutputCounts* outCounts)	PX_OVERRIDE;

		bool								isTGS() const { return mIsTGS; }
		bool								isExternalForcesEveryTgsIterationEnabled() const { return mIsExternalForcesEveryTgsIterationEnabled; }

		void								doPreIntegrationTaskCommon(ev4sio_physx::PxBaseTask* continuation);

		void 								doConstraintPrePrepCommon(ev4sio_physx::PxBaseTask* continuation);

		void								doConstraintPrePrepGPUCommon(bool hasForceThresholds);

		void								cpuJointPrePrepTask(ev4sio_physx::PxBaseTask* continuation);

		void 								allocateTempPinnedSolverMemoryCommon();

		PX_FORCE_INLINE bool				getEnableDirectGPUAPI() const { return mEnableDirectGPUAPI;	}

		PxvSimStats&			 			getSimStats() { return mSimStats; }

		PxBaseTask*								mLostTouchTask;

		PxU32									mTotalEdges;
		PxU32									mTotalPreviousEdges;

		PxsContactManagerOutputIterator			mOutputIterator;

		PxReal*									mGPURestDistances;
		ev4sio_Sc::ShapeInteraction**					mGPUShapeInteractions;
		PxsTorsionalFrictionData*				mGPUTorsionalData;

		ev4sio_Cm::FlushPool&							mFlushPool;
		bool									mSolvedThisFrame;
		PxgIncrementalPartition					mIncrementalPartition;

		ev4sio_Cm::PinnableArray<PxNodeIndex>			mActiveNodeIndex;	//this will change everyframe, include rigid bodies and articulation
		
		PxgSolverBody							mWorldSolverBody;
		PxgSolverBodyData						mWorldSolverBodyData;
		PxgSolverBodySleepData					mWorldSolverBodySleepData;
		PxgSolverTxIData						mWorldTxIData;

		ev4sio_Cm::PinnableArray<PxgSolverBody>		mSolverBodyPool;
		ev4sio_Cm::PinnableArray<PxAlignedTransform>	mBody2WorldPool;

		//write back from the active articulation
		//each articulation has max 64 links and max 3 * 63 dofsnd 1 wake counter
		//see PxgArticulationLinkJointRootStateData
		ev4sio_Cm::PinnableArray<PxU8>					mLinkAndJointAndRootStateDataPool;

		ev4sio_Cm::PinnableArray<PxgSolverBodySleepData>	mArticulationSleepDataPool;

		ev4sio_Cm::PinnableArray<PxU32>					m1dConstraintBatchIndices;
		ev4sio_Cm::PinnableArray<PxU32>					mContactConstraintBatchIndices;
		ev4sio_Cm::PinnableArray<PxU32>					mArti1dConstraintBatchIndices;
		ev4sio_Cm::PinnableArray<PxU32>					mArtiContactConstraintBatchIndices;

		ev4sio_Cm::PinnableArray<PxU32>					mConstraintsPerPartition;
		ev4sio_Cm::PinnableArray<PxU32>					mArtiConstraintsPerPartition;

		ev4sio_Cm::PinnableArray<PxgSolverBodyData>		mSolverBodyDataPool;
		ev4sio_Cm::PinnableArray<PxgSolverBodySleepData>	mSolverBodySleepDataPool;
		ev4sio_Cm::PinnableArray<PxgSolverTxIData>			mSolverTxIDataPool;

		PxgPinnedHostLinearMemoryAllocator*		mPinnedMemoryAllocator;

		PxgPinnedHostLinearMemoryAllocator*		mContactStreamAllocators[2];
		PxgPinnedHostLinearMemoryAllocator*		mPatchStreamAllocators[2];
		PxgPinnedHostLinearMemoryAllocator*		mForceStreamAllocator;
		PxgPinnedHostLinearMemoryAllocator*		mFrictionPatchStreamAllocator;

		PxU32									mCurrentContactStream;
#if PX_SUPPORT_OMNI_PVD
		OmniPvdContactStreams mOvdContactStreams;
#endif

		PxgIslandContext*						mIslandContextPool;
		PxU32									mNumIslandContextPool;

		PxU32									mNum1DConstraintBlockPrepPool; //this and mNum1dConstraintBatches is the same, we can get rid of it later

		PxU32									mNumContactManagers;
		PxU32									mNum1DConstraints;

		PxU32									mKinematicCount;
		PxU32									mArticulationCount;
		PxU32									mArticulationStartIndex; //record the start node index in the mActiveNodeIndex for articulation

		PxU32									mBodyCount;

		PxI32									mNumContactBatches;
		PxI32									mNum1dConstraintBatches;
		PxI32									mNumArtiContactBatches;
		PxI32									mNumArti1dConstraintBatches;
		PxI32									mNumStaticArtiContactBatches;
		PxI32									mNumStaticArti1dConstraintBatches;
		PxI32									mNumSelfArtiContactBatches;
		PxI32									mNumSelfArti1dConstraintBatches;

		PxI32									mNumStaticRigidContactBatches;
		PxI32									mNumStaticRigid1dConstraintBatches;

		PxI32									mArtiStaticConstraintBatchOffset;
		PxI32									mArtiStaticContactBatchOffset;

		//KS - we can't know this on CPU because the offset comes after the articulation constraints, which
		//are computed on GPU
		//PxI32									mRigidStaticConstraintBatchOffset;
		//PxI32									mRigidStaticContactBatchOffset;

		PxU32*									mConstraintUniqueIndices;
		PxU32*									mContactUniqueIndices;
		PxU32*									mArtiConstraintUniqueIndices;
		PxU32*									mArtiContactUniqueIndices;
		PxU32*									mArtiStaticConstraintUniqueIndices;
		PxU32*									mArtiStaticContactUniqueIndices;
		PxU32*									mArtiSelfConstraintUniqueIndices;
		PxU32*									mArtiSelfContactUniqueIndices;
		PxU32*									mArtiStaticConstraintStartIndex;

		PxU32*									mRigidStaticConstraintUniqueIndices;
		PxU32*									mRigidStaticContactUniqueIndices;

		PxU32*									mArtiStaticConstraintCount;
		PxU32*									mArtiStaticContactStartIndex;
		PxU32*									mArtiStaticContactCount;

		PxU32*									mRigidStaticConstraintStartIndex;
		PxU32*									mRigidStaticConstraintCount;

		PxI32									mCachedPositionIterations;
		PxI32									mCachedVelocityIterations;

		ev4sio_Cm::PinnableArray<PxU32>				mArtiStaticContactCounts;
		ev4sio_Cm::PinnableArray<PxU32>				mArtiStaticJointCounts;

		ev4sio_Cm::PinnableArray<PxU32>				mArtiStaticContactIndices;
		ev4sio_Cm::PinnableArray<PxU32>				mArtiStaticJointIndices;

		ev4sio_Cm::PinnableArray<PxU32>				mArtiSelfContactCounts;
		ev4sio_Cm::PinnableArray<PxU32>				mArtiSelfJointCounts;

		ev4sio_Cm::PinnableArray<PxU32>				mArtiSelfContactIndices;
		ev4sio_Cm::PinnableArray<PxU32>				mArtiSelfJointIndices;

		ev4sio_Cm::PinnableArray<PxU32>				mRigidStaticContactCounts;
		ev4sio_Cm::PinnableArray<PxU32>				mRigidStaticJointCounts;

		ev4sio_Cm::PinnableArray<PxU32>				mRigidStaticContactIndices;
		ev4sio_Cm::PinnableArray<PxU32>				mRigidStaticJointIndices;

		ev4sio_Cm::PinnableArray<PxU32>				mNodeIndicesStagingBuffer;
		ev4sio_Cm::PinnableArray<PxU32>				mIslandIds;
		ev4sio_Cm::PinnableArray<PxU32>				mIslandStaticTouchCounts;

		//other joint type(not d6) cpu constraints
		PxgConstraintBatchHeader*				mConstraintBatchHeaders;
		PxgConstraintBatchHeader*				mArticConstraintBatchHeaders;

		PxU32									mNumConstraintBatches;

		PxU32									mNumArticConstraintBatches;
		PxU32									mNumArtiStaticConstraintBatches;
		PxU32									mNumArtiSelfConstraintBatches;

		PxU32									mNumRigidStaticConstraintBatches;

		bool									mHasForceThresholds;
		const bool								mIsTGS;
		bool									mIsExternalForcesEveryTgsIterationEnabled;

		PxgCudaBroadPhaseSap*					mGpuBp;
		PxgGpuNarrowphaseCore*					mGpuNpCore;
		PxgArticulationCore*					mGpuArticulationCore;
		PxgSimulationCore*						mGpuSimulationCore;
		PxgSoftBodyCore*						mGpuSoftBodyCore;
		PxgFEMClothCore*						mGpuFEMClothCore;
		PxArray<PxgParticleSystemCore*>			mGpuParticleSystemCores;
		PxgParticleSystemCore*					mGpuPBDParticleSystemCore;
		
		PxgSolverCore*							mGpuSolverCore;

		PxU32									mMaxNumStaticPartitions;

		const bool								mEnableDirectGPUAPI;
		bool									mRecomputeArticulationBlockFormat;

		// when Direct GPU API is enabled, the constraint writeback data might have to be copied to host to
		// support breakable D6 joints
		bool									mEnforceConstraintWriteBackToHostCopy;

		PxgCpuPreIntegrationTask				mPreIntegrationTask;
		PxgCpuPrepTask							mPrepTask;
		PxgGpuPrePrepTask						mGpuPrePrepTask;
		PxgGpuIntegrationTask					mGpuIntegrationTask;
		PxgGpuTask								mGpuTask; //this task include preprepare constraint, prepare constraint, solve and integration tasks
		PxgPostSolveTask						mPostSolveTask;

		void									doConstraintPrepGPU();
		void									doPreIntegrationGPU(); //this is the pre integration code(copying data from pxgbodysim to solver body data)
		void									doArticulationGPU(); //this is the articulation forward dynamic code
		void									doSoftbodyGPU();//this is the soft body update tetrahedron rotations code
		void									doFEMClothGPU();
		void									doConstraintPrePrepGPU(); //this is the pre-prepare code for block format contacts and non-block format joints
	};
}

#endif


