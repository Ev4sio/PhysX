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

#include "ScPhysics.h"
#include "ScScene.h"
#include "BpBroadPhase.h"
#include "ScConstraintSim.h"
#include "ScConstraintCore.h"
#include "ScArticulationJointCore.h"
#include "ScArticulationTendonCore.h"
#include "ScArticulationMimicJointCore.h"
#include "ScArticulationSim.h"
#include "ScArticulationJointSim.h"
#include "ScArticulationTendonSim.h"
#include "ScArticulationMimicJointSim.h"
#include "ScConstraintInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScSimStats.h"
#include "PxvGlobals.h"
#include "PxsCCD.h"
#include "ScSimulationController.h"
#include "ScSqBoundsManager.h"
#include "DyIslandManager.h"

#if defined(__APPLE__) && defined(__POWERPC__)
	#include <ppc_intrinsics.h>
#endif

#if PX_SUPPORT_GPU_PHYSX
	#include "PxPhysXGpu.h"
	#include "PxsKernelWrangler.h"
	#include "PxsHeapMemoryAllocator.h"
	#include "cudamanager/PxCudaContextManager.h"
#endif

#include "PxsMemoryManager.h"

#include "ScShapeInteraction.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "PxDeformableSurface.h"
	#include "ScDeformableSurfaceSim.h"
	#include "DyDeformableSurface.h"
	#include "PxDeformableVolume.h"
	#include "ScDeformableVolumeSim.h"
	#include "DyDeformableVolume.h"
	#include "ScParticleSystemSim.h"
	#include "DyParticleSystem.h"
#endif

using namespace ev4sio_physx;
using namespace ev4sio_Cm;
using namespace ev4sio_Dy;
using namespace ev4sio_Sc;

PX_IMPLEMENT_OUTPUT_ERROR

namespace ev4sio_physx { 
namespace ev4sio_Sc {

class LLArticulationRCPool : public PxPool<FeatherstoneArticulation, PxAlignedAllocator<64> >
{
public:
	LLArticulationRCPool() {}
};

#if PX_SUPPORT_GPU_PHYSX

	class LLDeformableSurfacePool : public PxPool<DeformableSurface, PxAlignedAllocator<64> >
	{
	public:
		LLDeformableSurfacePool() {}
	};

	class LLDeformableVolumePool : public PxPool<DeformableVolume, PxAlignedAllocator<64> >
	{
	public:
		LLDeformableVolumePool() {}
	};

	class LLParticleSystemPool : public PxPool<ParticleSystem, PxAlignedAllocator<64> >
	{
	public:
		LLParticleSystemPool() {}
	};

#endif // PX_SUPPORT_GPU_PHYSX

static const char* sFilterShaderDataMemAllocId = "SceneDesc filterShaderData";

}}

void PxcDisplayContactCacheStats();

static const bool gUseNewTaskAllocationScheme = false;

namespace
{
	class ScAfterIntegrationTask : public ev4sio_Cm::Task
	{
	public:
		static const PxU32 MaxTasks = 256;
	private:
		const PxNodeIndex* const	mIndices;
		const PxU32					mNumBodies;
		PxsContext*					mContext;
		Context*					mDynamicsContext;
		PxsTransformCache&			mCache;
		ev4sio_Sc::Scene&					mScene;
	
	public:

		ScAfterIntegrationTask(const PxNodeIndex* const indices, PxU32 numBodies, PxsContext* context, Context* dynamicsContext, PxsTransformCache& cache, ev4sio_Sc::Scene& scene) :
			ev4sio_Cm::Task		(scene.getContextId()),
			mIndices		(indices),
			mNumBodies		(numBodies),
			mContext		(context),
			mDynamicsContext(dynamicsContext),
			mCache			(cache),
			mScene			(scene)
		{
		}

		virtual void runInternal()
		{		
			const PxU32 rigidBodyOffset = ev4sio_Sc::BodySim::getRigidBodyOffset();

			ev4sio_Sc::BodySim* bpUpdates[MaxTasks];
			ev4sio_Sc::BodySim* ccdBodies[MaxTasks];
			ev4sio_Sc::BodySim* activateBodies[MaxTasks];
			ev4sio_Sc::BodySim* deactivateBodies[MaxTasks];
			PxU32 nbBpUpdates = 0, nbCcdBodies = 0;

			ev4sio_IG::SimpleIslandManager& manager = *mScene.getSimpleIslandManager();
			const ev4sio_IG::IslandSim& islandSim = manager.getAccurateIslandSim();
			ev4sio_Bp::BoundsArray& boundsArray = mScene.getBoundsArray();

			ev4sio_Sc::BodySim* frozen[MaxTasks], * unfrozen[MaxTasks];
			PxU32 nbFrozen = 0, nbUnfrozen = 0;
			PxU32 nbActivated = 0, nbDeactivated = 0;

			for(PxU32 i = 0; i < mNumBodies; i++)
			{
				PxsRigidBody* rigid = getRigidBodyFromIG(islandSim, mIndices[i]);
				ev4sio_Sc::BodySim* bodySim = reinterpret_cast<ev4sio_Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
				
				PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();
				//If we got in this code, then this is an active object this frame. The solver computed the new wakeCounter and we 
				//commit it at this stage. We need to do it this way to avoid a race condition between the solver and the island gen, where
				//the island gen may have deactivated a body while the solver decided to change its wake counter.
				bodyCore.wakeCounter = bodyCore.solverWakeCounter;
				PxsRigidBody& llBody = bodySim->getLowLevelBody();

				const PxIntBool isFrozen = bodySim->isFrozen();
				if(!isFrozen)
				{
					bpUpdates[nbBpUpdates++] = bodySim;

					// PT: TODO: remove duplicate "isFrozen" test inside updateCached
	//				bodySim->updateCached(NULL);
					bodySim->updateCached(mCache, boundsArray);
				}

				if(llBody.isFreezeThisFrame() && isFrozen)
					frozen[nbFrozen++] = bodySim;
				else if(llBody.isUnfreezeThisFrame())
					unfrozen[nbUnfrozen++] = bodySim;

				if(bodyCore.mFlags & PxRigidBodyFlag::eENABLE_CCD)
					ccdBodies[nbCcdBodies++] = bodySim;

				if(llBody.isActivateThisFrame())
				{
					PX_ASSERT(!llBody.isDeactivateThisFrame());
					activateBodies[nbActivated++] = bodySim;
				}
				else if(llBody.isDeactivateThisFrame())
				{
					deactivateBodies[nbDeactivated++] = bodySim;
				}
				llBody.clearAllFrameFlags();
			}
			if(nbBpUpdates)
			{
				mCache.setChangedState();
				boundsArray.setChangedState();
			}

			if(nbBpUpdates>0 || nbFrozen > 0 || nbCcdBodies>0 || nbActivated>0 || nbDeactivated>0)
			{
				//Write active bodies to changed actor map
				mContext->getLock().lock();
				PxBitMapPinned& changedAABBMgrHandles = mScene.getAABBManager()->getChangedAABBMgActorHandleMap();
			
				for(PxU32 i = 0; i < nbBpUpdates; i++)
				{
					// PT: ### changedMap pattern #1
					PxU32 nbElems = bpUpdates[i]->getNbElements();
					ev4sio_Sc::ElementSim** elems = bpUpdates[i]->getElements();
					while (nbElems--)
					{
						ev4sio_Sc::ShapeSim* sim = static_cast<ev4sio_Sc::ShapeSim*>(*elems++);
						// PT: TODO: what's the difference between this test and "isInBroadphase" as used in bodySim->updateCached ?
						// PT: Also, shouldn't it be "isInAABBManager" rather than BP ?
						if (sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))	// TODO: need trigger shape here?
							changedAABBMgrHandles.growAndSet(sim->getElementID());
					}
				}

				PxArray<ev4sio_Sc::BodySim*>& sceneCcdBodies = mScene.getCcdBodies();
				for (PxU32 i = 0; i < nbCcdBodies; i++)
					sceneCcdBodies.pushBack(ccdBodies[i]);

				for(PxU32 i=0;i<nbFrozen;i++)
				{
					PX_ASSERT(frozen[i]->isFrozen());
					frozen[i]->freezeTransforms(&changedAABBMgrHandles);
				}

				for(PxU32 i=0;i<nbUnfrozen;i++)
				{
					PX_ASSERT(!unfrozen[i]->isFrozen());
					unfrozen[i]->createSqBounds();
				}
			
				for(PxU32 i = 0; i < nbActivated; ++i)
					activateBodies[i]->notifyNotReadyForSleeping();

				for(PxU32 i = 0; i < nbDeactivated; ++i)
					deactivateBodies[i]->notifyReadyForSleeping();

				mContext->getLock().unlock();
			}
		}

		virtual const char* getName() const
		{
			return "ScScene.afterIntegrationTask";
		}

	private:
		PX_NOCOPY(ScAfterIntegrationTask)
	};

	class ScSimulationControllerCallback : public PxsSimulationControllerCallback
	{
		ev4sio_Sc::Scene* mScene; 
	public:

		ScSimulationControllerCallback(ev4sio_Sc::Scene* scene) : mScene(scene)
		{
		}
	
		virtual void updateScBodyAndShapeSim(PxBaseTask* continuation)	PX_OVERRIDE
		{
			PxsContext* contextLL = mScene->getLowLevelContext();
			ev4sio_IG::SimpleIslandManager* islandManager = mScene->getSimpleIslandManager();
			ev4sio_Dy::Context* dynamicContext = mScene->getDynamicsContext();

			ev4sio_Cm::FlushPool& flushPool = contextLL->getTaskPool();

			const PxU32 MaxBodiesPerTask = ScAfterIntegrationTask::MaxTasks;

			PxsTransformCache& cache = contextLL->getTransformCache();

			const ev4sio_IG::IslandSim& islandSim = islandManager->getAccurateIslandSim();

			/*const*/ PxU32 numBodies = islandSim.getNbActiveNodes(ev4sio_IG::Node::eRIGID_BODY_TYPE);

			const PxNodeIndex*const nodeIndices = islandSim.getActiveNodes(ev4sio_IG::Node::eRIGID_BODY_TYPE);

			const PxU32 rigidBodyOffset = ev4sio_Sc::BodySim::getRigidBodyOffset();

			// PT: TASK-CREATION TAG
			if(!gUseNewTaskAllocationScheme)
			{
				PxU32 nbShapes = 0;
				PxU32 startIdx = 0;
				for (PxU32 i = 0; i < numBodies; i++)
				{
					if (nbShapes >= MaxBodiesPerTask)
					{
						ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices + startIdx, i - startIdx,
							contextLL, dynamicContext, cache, *mScene));

						startTask(task, continuation);

						startIdx = i;
						nbShapes = 0;
					}
					PxsRigidBody* rigid = getRigidBodyFromIG(islandSim, nodeIndices[i]);
					ev4sio_Sc::BodySim* bodySim = reinterpret_cast<ev4sio_Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
					nbShapes += PxMax(1u, bodySim->getNbShapes()); //Always add at least 1 shape in, even if the body has zero shapes because there is still some per-body overhead
				}

				if (nbShapes)
				{
					ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices + startIdx, numBodies - startIdx,
						contextLL, dynamicContext, cache, *mScene));

					startTask(task, continuation);
				}
			}
			else
			{
				// PT:
				const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();

				PxU32 nbPerTask;
				if(numCpuTasks)
					nbPerTask = numBodies > numCpuTasks ? numBodies / numCpuTasks : numBodies;
				else
					nbPerTask = numBodies;

				// PT: we need to respect that limit even with a single thread, because of hardcoded buffer limits in ScAfterIntegrationTask.
				if(nbPerTask>MaxBodiesPerTask)
					nbPerTask = MaxBodiesPerTask;

				PxU32 start = 0;
				while(numBodies)
				{
					const PxU32 nb = numBodies < nbPerTask ? numBodies : nbPerTask;

					ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices+start, nb, 
						contextLL, dynamicContext, cache, *mScene));

					start += nb;
					numBodies -= nb;

					startTask(task, continuation);
				}
			}
		}

		virtual PxU32 getNbCcdBodies()	PX_OVERRIDE
		{ 
			return mScene->getCcdBodies().size(); 
		}
	};

	// PT: TODO: what is this Pxg class doing here?
	class PxgUpdateBodyAndShapeStatusTask : public ev4sio_Cm::Task
	{
	public:
		static const PxU32 MaxTasks = 2048;
	private:
		const PxNodeIndex* const mNodeIndices;
		const PxU32 mNumBodies;
		ev4sio_Sc::Scene& mScene;
		void**	mRigidBodyLL;
		PxU32*	mActivatedBodies;
		PxU32*	mDeactivatedBodies;
		PxI32&	mCCDBodyWriteIndex;
	
	public:

		PxgUpdateBodyAndShapeStatusTask(const PxNodeIndex* const indices, PxU32 numBodies, void** rigidBodyLL, PxU32* activatedBodies, PxU32* deactivatedBodies, ev4sio_Sc::Scene& scene, PxI32& ccdBodyWriteIndex) : 
			ev4sio_Cm::Task			(scene.getContextId()),
			mNodeIndices		(indices),
			mNumBodies			(numBodies),
			mScene				(scene),
			mRigidBodyLL		(rigidBodyLL),
			mActivatedBodies	(activatedBodies),  
			mDeactivatedBodies	(deactivatedBodies),
			mCCDBodyWriteIndex	(ccdBodyWriteIndex)
		{
		}

		virtual void runInternal()
		{		
			ev4sio_IG::SimpleIslandManager& islandManager = *mScene.getSimpleIslandManager();
			const ev4sio_IG::IslandSim& islandSim = islandManager.getAccurateIslandSim();

			PxU32 nbCcdBodies = 0;

			PxArray<ev4sio_Sc::BodySim*>& sceneCcdBodies = mScene.getCcdBodies();
			ev4sio_Sc::BodySim* ccdBodies[MaxTasks];

			const size_t bodyOffset =  PX_OFFSET_OF_RT(ev4sio_Sc::BodySim, getLowLevelBody());

			for(PxU32 i=0; i<mNumBodies; ++i)
			{
				const PxU32 nodeIndex = mNodeIndices[i].index();
				PxsRigidBody* rigidLL = reinterpret_cast<PxsRigidBody*>(mRigidBodyLL[nodeIndex]);

				PxsBodyCore* bodyCore = &rigidLL->getCore();
				bodyCore->wakeCounter = bodyCore->solverWakeCounter;
				//we can set the frozen/unfrozen flag in GPU, but we have copied the internalflags
				//from the solverbodysleepdata to pxsbodycore, so we just need to clear the frozen flag in here
				rigidLL->clearAllFrameFlags();

				PX_ASSERT(mActivatedBodies[nodeIndex] <= 1);
				PX_ASSERT(mDeactivatedBodies[nodeIndex] <= 1);
				if(mActivatedBodies[nodeIndex])
				{
					PX_ASSERT(bodyCore->wakeCounter > 0.0f);
					islandManager.activateNode(mNodeIndices[i]);
				}
				else if(mDeactivatedBodies[nodeIndex])
				{
					//KS - the CPU code can reset the wake counter due to lost touches in parallel with the solver, so we need to verify
					//that the wakeCounter is still 0 before deactivating the node
					if (bodyCore->wakeCounter == 0.0f)
					{
						islandManager.deactivateNode(mNodeIndices[i]);
					}
				}

				if (bodyCore->mFlags & PxRigidBodyFlag::eENABLE_CCD)
				{
					PxsRigidBody* rigidBody = getRigidBodyFromIG(islandSim, mNodeIndices[i]);
					ev4sio_Sc::BodySim* bodySim = reinterpret_cast<ev4sio_Sc::BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - bodyOffset);
					ccdBodies[nbCcdBodies++] = bodySim;
				}
			}
			if(nbCcdBodies > 0)
			{
				PxI32 startIndex = PxAtomicAdd(&mCCDBodyWriteIndex, PxI32(nbCcdBodies)) - PxI32(nbCcdBodies);
				for(PxU32 a = 0; a < nbCcdBodies; ++a)
				{
					sceneCcdBodies[startIndex + a] = ccdBodies[a];
				}
			}
		}

		virtual const char* getName() const
		{
			return "ScScene.PxgUpdateBodyAndShapeStatusTask";
		}

	private:
		PX_NOCOPY(PxgUpdateBodyAndShapeStatusTask)
	};

#if PX_SUPPORT_GPU_PHYSX
	// PT: TODO: what is this Pxg class doing here?
	class PxgSimulationControllerCallback : public PxsSimulationControllerCallback
	{
		ev4sio_Sc::Scene* mScene; 
		PxI32 mCcdBodyWriteIndex;

	public:
		PxgSimulationControllerCallback(ev4sio_Sc::Scene* scene) : mScene(scene), mCcdBodyWriteIndex(0)
		{
		}

		virtual void updateScBodyAndShapeSim(PxBaseTask* continuation)	PX_OVERRIDE
		{
			ev4sio_IG::SimpleIslandManager* islandManager = mScene->getSimpleIslandManager();
			PxsSimulationController* simulationController = mScene->getSimulationController();
			PxsContext*	contextLL = mScene->getLowLevelContext();
			ev4sio_IG::IslandSim& islandSim = islandManager->getAccurateIslandSim();
			const PxU32 numBodies = islandSim.getNbActiveNodes(ev4sio_IG::Node::eRIGID_BODY_TYPE);
			const PxNodeIndex*const nodeIndices = islandSim.getActiveNodes(ev4sio_IG::Node::eRIGID_BODY_TYPE);

			PxU32* activatedBodies = simulationController->getActiveBodies();
			PxU32* deactivatedBodies = simulationController->getDeactiveBodies();

			//PxsRigidBody** rigidBodyLL = simulationController->getRigidBodies();
			void** rigidBodyLL = simulationController->getRigidBodies();

			ev4sio_Cm::FlushPool& flushPool = contextLL->getTaskPool();

			PxArray<ev4sio_Sc::BodySim*>& ccdBodies = mScene->getCcdBodies();
			ccdBodies.forceSize_Unsafe(0);
			ccdBodies.reserve(numBodies);
			ccdBodies.forceSize_Unsafe(numBodies);

			mCcdBodyWriteIndex = 0;

			// PT: TASK-CREATION TAG
			for(PxU32 i = 0; i < numBodies; i+=PxgUpdateBodyAndShapeStatusTask::MaxTasks)
			{
				PxgUpdateBodyAndShapeStatusTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgUpdateBodyAndShapeStatusTask)), PxgUpdateBodyAndShapeStatusTask(nodeIndices + i, 
					PxMin(PxgUpdateBodyAndShapeStatusTask::MaxTasks, numBodies - i), rigidBodyLL, activatedBodies, deactivatedBodies, *mScene, mCcdBodyWriteIndex));
				task->setContinuation(continuation);
				task->removeReference();
			}
		
			const PxU32 nbFrozenShapes = simulationController->getNbFrozenShapes();
			const PxU32 nbUnfrozenShapes = simulationController->getNbUnfrozenShapes();

			if(nbFrozenShapes || nbUnfrozenShapes)
			{
				PxU32* unfrozenShapeIndices = simulationController->getUnfrozenShapes();
				PxU32* frozenShapeIndices = simulationController->getFrozenShapes();

				ev4sio_Sc::ShapeSimBase** shapeSimsLL = simulationController->getShapeSims();
	
				for(PxU32 i=0; i<nbFrozenShapes; ++i)
				{
					ev4sio_Sc::ShapeSimBase* shape = shapeSimsLL[frozenShapeIndices[i]];
					PX_ASSERT(shape);
					shape->destroySqBounds();
				}

				for(PxU32 i=0; i<nbUnfrozenShapes; ++i)
				{
					ev4sio_Sc::ShapeSimBase* shape = shapeSimsLL[unfrozenShapeIndices[i]];
					PX_ASSERT(shape);
					shape->createSqBounds();
				}
			}

			if (simulationController->hasDeformableSurfaces())
			{
				//KS - technically, there's a race condition calling activateNode/deactivateNode, but we know that it is 
				//safe because these deactivate/activate calls came from the solver. This means that we know that the 
				//actors are active currently, so at most we are just clearing/setting the ready for sleeping flag.
				//None of the more complex logic that touching shared state will be executed.
				const PxU32 nbActivatedSurfaces = simulationController->getNbActivatedDeformableSurfaces();
				ev4sio_Dy::DeformableSurface** activatedSurfaces = simulationController->getActivatedDeformableSurfaces();
				for (PxU32 i = 0; i < nbActivatedSurfaces; ++i)
				{
					PxNodeIndex nodeIndex = activatedSurfaces[i]->getSim()->getNodeIndex();
					islandManager->activateNode(nodeIndex);
				}

				const PxU32 nbDeactivatedSurfaces = simulationController->getNbDeactivatedDeformableSurfaces();
				ev4sio_Dy::DeformableSurface** deactivatedSurfaces = simulationController->getDeactivatedDeformableSurfaces();
				for (PxU32 i = 0; i < nbDeactivatedSurfaces; ++i)
				{
					PxNodeIndex nodeIndex = deactivatedSurfaces[i]->getSim()->getNodeIndex();
					islandManager->deactivateNode(nodeIndex);
				}
			}

			if (simulationController->hasDeformableVolumes())
			{
				//KS - technically, there's a race condition calling activateNode/deactivateNode, but we know that it is 
				//safe because these deactivate/activate calls came from the solver. This means that we know that the 
				//actors are active currently, so at most we are just clearing/setting the ready for sleeping flag.
				//None of the more complex logic that touching shared state will be executed.

				const PxU32 nbDeactivatedVolumes = simulationController->getNbDeactivatedDeformableVolumes();
				ev4sio_Dy::DeformableVolume** deactivatedVolumes = simulationController->getDeactivatedDeformableVolumes();
				for (PxU32 i = 0; i < nbDeactivatedVolumes; ++i)
				{
					PxNodeIndex nodeIndex = deactivatedVolumes[i]->getSim()->getNodeIndex();
					islandManager->deactivateNode(nodeIndex);
				}

				const PxU32 nbActivatedVolumes = simulationController->getNbActivatedDeformableVolumes();
				ev4sio_Dy::DeformableVolume** activatedVolumes = simulationController->getActivatedDeformableVolumes();
				for (PxU32 i = 0; i < nbActivatedVolumes; ++i)
				{
					PxNodeIndex nodeIndex = activatedVolumes[i]->getSim()->getNodeIndex();
					islandManager->activateNode(nodeIndex);
				}
			}
		}

		virtual PxU32	getNbCcdBodies()	PX_OVERRIDE
		{
			return PxU32(mCcdBodyWriteIndex);
		}
	};
#endif
}

static ev4sio_Bp::AABBManagerBase* createAABBManagerCPU(const PxSceneDesc& desc, ev4sio_Bp::BroadPhase* broadPhase, ev4sio_Bp::BoundsArray* boundsArray, PxFloatArrayPinned* contactDistances, PxVirtualAllocator& allocator, PxU64 contextID)
{
	return PX_NEW(ev4sio_Bp::AABBManager)(*broadPhase, *boundsArray, *contactDistances,
		desc.limits.maxNbAggregates, desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes, allocator, contextID,
		desc.kineKineFilteringMode, desc.staticKineFilteringMode);
}

#if PX_SUPPORT_GPU_PHYSX
static ev4sio_Bp::AABBManagerBase* createAABBManagerGPU(PxsKernelWranglerManager* kernelWrangler, PxCudaContextManager* cudaContextManager, PxsHeapMemoryAllocatorManager* heapMemoryAllocationManager,
												const PxSceneDesc& desc, ev4sio_Bp::BroadPhase* broadPhase, ev4sio_Bp::BoundsArray* boundsArray, PxFloatArrayPinned* contactDistances, PxVirtualAllocator& allocator, PxU64 contextID)
{
	return PxvGetPhysXGpu(true)->createGpuAABBManager(
		kernelWrangler,
		cudaContextManager,
		desc.gpuComputeVersion,
		desc.gpuDynamicsConfig,
		heapMemoryAllocationManager,
		*broadPhase, *boundsArray, *contactDistances,
		desc.limits.maxNbAggregates, desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes, allocator, contextID,
		desc.kineKineFilteringMode, desc.staticKineFilteringMode);
}
#endif

ev4sio_Sc::Scene::Scene(const PxSceneDesc& desc, PxU64 contextID) :
	mContextId						(contextID),
	mActiveBodies					("sceneActiveBodies"),
	mActiveKinematicBodyCount		(0),
	mActiveDynamicBodyCount			(0),
	mActiveKinematicsCopy			(NULL),
	mActiveKinematicsCopyCapacity	(0),
	mPointerBlock8Pool				("scenePointerBlock8Pool"),
	mPointerBlock16Pool				("scenePointerBlock16Pool"),
	mPointerBlock32Pool				("scenePointerBlock32Pool"),
	mLLContext						(NULL),
	mAABBManager					(NULL),
	mCCDContext						(NULL),
	mNumFastMovingShapes			(0),
	mCCDPass						(0),
	mSimpleIslandManager			(NULL),
	mDynamicsContext				(NULL),
	mMemoryManager					(NULL),
#if PX_SUPPORT_GPU_PHYSX
	mGpuWranglerManagers			(NULL),
	mHeapMemoryAllocationManager	(NULL),
#endif
	mSimulationController			(NULL),
	mSimulationControllerCallback	(NULL),
	mGravity						(PxVec3(0.0f)),
	mDt								(0),
	mOneOverDt						(0),
	mTimeStamp						(1),		// PT: has to start to 1 to fix determinism bug. I don't know why yet but it works.
	mReportShapePairTimeStamp		(0),
	mTriggerBufferAPI				("sceneTriggerBufferAPI"),
	mArticulations					("sceneArticulations"),
	mBrokenConstraints				("sceneBrokenConstraints"),
	mActiveBreakableConstraints		("sceneActiveBreakableConstraints"),
	mMemBlock128Pool				("PxsContext ConstraintBlock128Pool"),
	mMemBlock256Pool				("PxsContext ConstraintBlock256Pool"),
	mMemBlock384Pool				("PxsContext ConstraintBlock384Pool"),
	mMemBlock512Pool				("PxsContext ConstraintBlock512Pool"),
	mNPhaseCore						(NULL),
	mKineKineFilteringMode			(desc.kineKineFilteringMode),
	mStaticKineFilteringMode		(desc.staticKineFilteringMode),
	mSleepBodies					("sceneSleepBodies"),
	mWokeBodies						("sceneWokeBodies"),
	mEnableStabilization			(desc.flags & PxSceneFlag::eENABLE_STABILIZATION),
	mActiveActors					("clientActiveActors"),
	mFrozenActors					("clientFrozenActors"),
	mClientPosePreviewBodies		("clientPosePreviewBodies"),
	mClientPosePreviewBuffer		("clientPosePreviewBuffer"),
	mSimulationEventCallback		(NULL),
	mInternalFlags					(SceneInternalFlag::eSCENE_DEFAULT),
	mPublicFlags					(desc.flags),
	mAnchorCore						(PxTransform(PxIdentity)),
	mStaticAnchor					(NULL),
	mConstraintSimPool				("ScScene::ConstraintSim"),
	mConstraintInteractionPool		("ScScene::ConstraintInteraction"),
	mBatchRemoveState				(NULL),
	mLostTouchPairs					("sceneLostTouchPairs"),
	mVisualizationParameterChanged	(false),
	mMaxNbArticulationLinks			(0),
	mNbRigidStatics					(0),
	mNbRigidDynamics				(0),
	mNbRigidKinematic				(0),
	mSecondPassNarrowPhase			(contextID, this, "ScScene.secondPassNarrowPhase"),
	mPostNarrowPhase				(contextID, this, "ScScene.postNarrowPhase"),
	mFinalizationPhase				(contextID, this, "ScScene.finalizationPhase"),
	mUpdateCCDMultiPass				(contextID, this, "ScScene.updateCCDMultiPass"),
	mAfterIntegration				(contextID, this, "ScScene.afterIntegration"),
	mPostSolver						(contextID, this, "ScScene.postSolver"),
	mSolver							(contextID, this, "ScScene.rigidBodySolver"),
	mUpdateBodies					(contextID, this, "ScScene.updateBodies"),
	mUpdateShapes					(contextID, this, "ScScene.updateShapes"),
	mUpdateSimulationController		(contextID, this, "ScScene.updateSimulationController"),
	mUpdateDynamics					(contextID, this, "ScScene.updateDynamics"),
	mUpdateDynamicsPostPartitioning	(contextID, this, "ScScene.updateDynamicsPostPartitioning"),
	mProcessLostContactsTask		(contextID, this, "ScScene.processLostContact"),
	mProcessLostContactsTask2		(contextID, this, "ScScene.processLostContact2"),
	mProcessLostContactsTask3		(contextID, this, "ScScene.processLostContact3"),
	mDestroyManagersTask			(contextID, this, "ScScene.destroyManagers"),
	mLostTouchReportsTask			(contextID, this, "ScScene.lostTouchReports"),
	mUnregisterInteractionsTask		(contextID, this, "ScScene.unregisterInteractions"),
	mProcessNarrowPhaseLostTouchTasks(contextID, this, "ScScene.processNpLostTouchTask"),
	mProcessNPLostTouchEvents		(contextID, this, "ScScene.processNPLostTouchEvents"),
	mPostThirdPassIslandGenTask		(contextID, this, "ScScene.postThirdPassIslandGenTask"),
#if !USE_SPLIT_SECOND_PASS_ISLAND_GEN
	mPostIslandGen					(contextID, this, "ScScene.postIslandGen"),
#endif
	mIslandGen						(contextID, this, "ScScene.islandGen"),
	mPreRigidBodyNarrowPhase		(contextID, this, "ScScene.preRigidBodyNarrowPhase"),
	mSetEdgesConnectedTask			(contextID, this, "ScScene.setEdgesConnectedTask"),
	mUpdateBoundAndShapeTask		(contextID, this, "ScScene.updateBoundsAndShapesTask"),
	mRigidBodyNarrowPhase			(contextID, this, "ScScene.rigidBodyNarrowPhase"),
	mRigidBodyNPhaseUnlock			(contextID, this, "ScScene.unblockNarrowPhase"),
	mPostBroadPhase					(contextID, this, "ScScene.postBroadPhase"),
	mPostBroadPhaseCont				(contextID, this, "ScScene.postBroadPhaseCont"),
	mPostBroadPhase2				(contextID, this, "ScScene.postBroadPhase2"),
	mPostBroadPhase3				(contextID, this, "ScScene.postBroadPhase3"),
	mPreallocateContactManagers		(contextID, this, "ScScene.preallocateContactManagers"),
	mIslandInsertion				(contextID, this, "ScScene.islandInsertion"),
	mRegisterContactManagers		(contextID, this, "ScScene.registerContactManagers"),
	mRegisterInteractions			(contextID, this, "ScScene.registerInteractions"),
	mRegisterSceneInteractions		(contextID, this, "ScScene.registerSceneInteractions"),
	mBroadPhase						(contextID, this, "ScScene.broadPhase"),
	mAdvanceStep					(contextID, this, "ScScene.advanceStep"),
	mCollideStep					(contextID, this, "ScScene.collideStep"),	
	mBpFirstPass					(contextID, this, "ScScene.broadPhaseFirstPass"),
	mBpSecondPass					(contextID, this, "ScScene.broadPhaseSecondPass"),
	mBpUpdate						(contextID, this, "ScScene.updateBroadPhase"),
	mPreIntegrate                   (contextID, this, "ScScene.preIntegrate"),
	mTaskPool						(16384),
	mTaskManager					(NULL),
	mCudaContextManager				(desc.cudaContextManager),
	mContactReportsNeedPostSolverVelocity(false),
	mUseGpuDynamics					(false),
	mUseGpuBp						(false),
	mCCDBp							(false),
	mSimulationStage				(SimulationStage::eCOMPLETE),
	mPosePreviewBodies				("scenePosePreviewBodies"),
	mOverlapFilterTaskHead			(NULL),
	mOverlapCreatedTaskHead			(NULL),
	mIslandInsertionTaskHead		(NULL),
	mIsCollisionPhaseActive			(false),
	mIsDirectGPUAPIInitialized		(false),
	mResidual						(),
	mOnSleepingStateChanged			(NULL)
#if PX_SUPPORT_GPU_PHYSX
	,mDeformableSurfaces			("sceneDeformableSurfaces"), 
	mDeformableVolumes				("sceneDeformableVolumes"),
	mParticleSystems				("sceneParticleSystems")
#endif
{
#if PX_SUPPORT_GPU_PHYSX
	mLLDeformableSurfacePool	= PX_NEW(LLDeformableSurfacePool);
	mLLDeformableVolumePool		= PX_NEW(LLDeformableVolumePool);
	mLLParticleSystemPool		= PX_NEW(LLParticleSystemPool);

	mWokeDeformableVolumeListValid = true;
	mSleepDeformableVolumeListValid = true;
#endif

	for(PxU32 type = 0; type < InteractionType::eTRACKED_IN_SCENE_COUNT; ++type)
		mInteractions[type].reserve(64);

	for (int i=0; i < InteractionType::eTRACKED_IN_SCENE_COUNT; ++i)
		mActiveInteractionCount[i] = 0;

	mStats						= PX_NEW(SimStats);
	mConstraintIDTracker		= PX_NEW(ObjectIDTracker);
	mActorIDTracker				= PX_NEW(ObjectIDTracker);
	mElementIDPool				= PX_NEW(ObjectIDTracker);

	mTriggerBufferExtraData		= reinterpret_cast<TriggerBufferExtraData*>(PX_ALLOC(sizeof(TriggerBufferExtraData), "ScScene::TriggerBufferExtraData"));
	PX_PLACEMENT_NEW(mTriggerBufferExtraData, TriggerBufferExtraData("ScScene::TriggerPairExtraData"));

	mStaticSimPool				= PX_NEW(PreallocatingPool<StaticSim>)(64, "StaticSim");
	mBodySimPool				= PX_NEW(PreallocatingPool<BodySim>)(64, "BodySim");
	mShapeSimPool				= PX_NEW(PreallocatingPool<ShapeSim>)(128, "ShapeSim");
	mLLArticulationRCPool		= PX_NEW(LLArticulationRCPool);
	mSimStateDataPool			= PX_NEW(PxPool<SimStateData>)("ScScene::SimStateData");

	mSqBoundsManager			= PX_NEW(SqBoundsManager);

	mTaskManager				= PxTaskManager::createTaskManager(*ev4sio_PxGetErrorCallback(), desc.cpuDispatcher);

	for(PxU32 i=0; i<PxGeometryType::eGEOMETRY_COUNT; i++)
		mNbGeometries[i] = 0;

	bool useGpuDynamics = false;
	bool useGpuBroadphase = false;

#if PX_SUPPORT_GPU_PHYSX
	if(desc.flags & PxSceneFlag::eENABLE_GPU_DYNAMICS)
	{
		if(!mCudaContextManager)
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "GPU solver pipeline failed, switching to software");
		else if(mCudaContextManager->supportsArchSM30())
			useGpuDynamics = true;
	}

	if(desc.broadPhaseType == PxBroadPhaseType::eGPU)
	{
		if(!mCudaContextManager)
			outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "GPU Bp pipeline failed, switching to software");
		else if(mCudaContextManager->supportsArchSM30())
			useGpuBroadphase = true;
	}
#endif

	mUseGpuDynamics = useGpuDynamics;
	mUseGpuBp = useGpuBroadphase;

	mLLContext = PX_NEW(PxsContext)(desc, mTaskManager, mTaskPool, mCudaContextManager, desc.contactPairSlabSize, contextID);
	
	if (mLLContext == 0)
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Failed to create context!");
		return;
	}
	mLLContext->setMaterialManager(&getMaterialManager());

#if PX_SUPPORT_GPU_PHYSX
	if (useGpuBroadphase || useGpuDynamics)
	{
		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);

		// PT: this creates a PxgMemoryManager, whose host memory allocator is a PxgCudaHostMemoryAllocatorCallback
		mMemoryManager = physxGpu->createGpuMemoryManager(mLLContext->getCudaContextManager());
		mGpuWranglerManagers = physxGpu->getGpuKernelWranglerManager(mLLContext->getCudaContextManager());
		// PT: this creates a PxgHeapMemoryAllocatorManager
		mHeapMemoryAllocationManager = physxGpu->createGpuHeapMemoryAllocatorManager(desc.gpuDynamicsConfig.heapCapacity, mMemoryManager, desc.gpuComputeVersion);
	}
	else
#endif
	{
		// PT: this creates a PxsDefaultMemoryManager
		mMemoryManager = createDefaultMemoryManager();
	}

	ev4sio_Bp::BroadPhase* broadPhase = NULL;

	//Note: broadphase should be independent of AABBManager.  MBP uses it to call getBPBounds but it has 
	//already been passed all bounds in BroadPhase::update() so should use that instead.
	// PT: above comment is obsolete: MBP now doesn't call getBPBounds anymore (except in commented out code)
	// and it is instead the GPU broadphase which is not independent from the GPU AABB manager.......
	if(!useGpuBroadphase)
	{
		PxBroadPhaseType::Enum broadPhaseType = desc.broadPhaseType;

		if (broadPhaseType == PxBroadPhaseType::eGPU)
			broadPhaseType = PxBroadPhaseType::eABP;

		broadPhase = ev4sio_Bp::BroadPhase::create(
			broadPhaseType, 
			desc.limits.maxNbRegions, 
			desc.limits.maxNbBroadPhaseOverlaps, 
			desc.limits.maxNbStaticShapes, 
			desc.limits.maxNbDynamicShapes,
			contextID);
	}
#if PX_SUPPORT_GPU_PHYSX
	else
	{
		broadPhase = PxvGetPhysXGpu(true)->createGpuBroadPhase(	mGpuWranglerManagers, mLLContext->getCudaContextManager(),
																desc.gpuComputeVersion, desc.gpuDynamicsConfig,
																mHeapMemoryAllocationManager, contextID);
	}
#endif

	//create allocator
	PxVirtualAllocatorCallback* allocatorCallback = mMemoryManager->getHostMemoryAllocator();
	PxVirtualAllocator allocator(allocatorCallback);

#if PX_SUPPORT_GPU_PHYSX
	const bool directAPI = mPublicFlags & PxSceneFlag::eENABLE_DIRECT_GPU_API;
	if(directAPI)
	{
		mBoundsArray = PxvGetPhysXGpu(true)->createGpuBounds(allocator);
	}
	else
#endif
	{
		mBoundsArray = PX_NEW(ev4sio_Bp::BoundsArray)(allocator);
	}
	
	mContactDistance = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxFloatArrayPinned), "ContactDistance"), PxFloatArrayPinned)(allocator);
	mHasContactDistanceChanged = false;

	const bool useEnhancedDeterminism = mPublicFlags & PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;

	mSimpleIslandManager = PX_NEW(ev4sio_IG::SimpleIslandManager)(useEnhancedDeterminism, useGpuBroadphase || useGpuDynamics, contextID);
	PX_ASSERT(mSimpleIslandManager);

	PxvNphaseImplementationFallback* cpuNphaseImplementation = createNphaseImplementationContext(*mLLContext, &mSimpleIslandManager->getAccurateIslandSim(), allocatorCallback, useGpuDynamics);

	if (!useGpuDynamics)
	{
		if (desc.solverType == PxSolverType::ePGS)
		{
			mDynamicsContext = createDynamicsContext
			(&mLLContext->getNpMemBlockPool(), mLLContext->getScratchAllocator(),
				mLLContext->getTaskPool(), mLLContext->getSimStats(), &mLLContext->getTaskManager(), allocatorCallback, &getMaterialManager(),
				*mSimpleIslandManager, contextID, mEnableStabilization, useEnhancedDeterminism, desc.maxBiasCoefficient,
				desc.flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION, desc.getTolerancesScale().length,
				desc.flags & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING);
		}
		else
		{
			mDynamicsContext = createTGSDynamicsContext
			(&mLLContext->getNpMemBlockPool(), mLLContext->getScratchAllocator(),
				mLLContext->getTaskPool(), mLLContext->getSimStats(), &mLLContext->getTaskManager(), allocatorCallback, &getMaterialManager(),
				*mSimpleIslandManager, contextID, mEnableStabilization, useEnhancedDeterminism,
				desc.getTolerancesScale().length, desc.flags & PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS,
				desc.flags & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING);
		}

		mLLContext->setNphaseImplementationContext(cpuNphaseImplementation);

		mSimulationControllerCallback = PX_NEW(ScSimulationControllerCallback)(this);
		mSimulationController = PX_NEW(SimulationController)(mSimulationControllerCallback);

		if (!useGpuBroadphase)
			mAABBManager = createAABBManagerCPU(desc, broadPhase, mBoundsArray, mContactDistance, allocator, contextID);
#if PX_SUPPORT_GPU_PHYSX
		else
			mAABBManager = createAABBManagerGPU(mGpuWranglerManagers, mLLContext->getCudaContextManager(), mHeapMemoryAllocationManager, desc, broadPhase, mBoundsArray, mContactDistance, allocator, contextID);
#endif
	}
	else
	{
#if PX_SUPPORT_GPU_PHYSX
		const bool enableBodyAccelerations = mPublicFlags & PxSceneFlag::eENABLE_BODY_ACCELERATIONS;

		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);

		// PT: why are we using mPublicFlags in one case and desc in other cases?

		mDynamicsContext = physxGpu->createGpuDynamicsContext(mLLContext->getTaskPool(), mGpuWranglerManagers, mLLContext->getCudaContextManager(),
			desc.gpuDynamicsConfig, *mSimpleIslandManager, desc.gpuMaxNumPartitions, desc.gpuMaxNumStaticPartitions, mEnableStabilization, useEnhancedDeterminism, 
			desc.maxBiasCoefficient, desc.gpuComputeVersion, mLLContext->getSimStats(), mHeapMemoryAllocationManager, 
			desc.flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION, desc.flags & PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS,
			desc.solverType, desc.getTolerancesScale().length, directAPI, contextID, desc.flags & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING);

		void* contactStreamBase = NULL;
		void* patchStreamBase = NULL;
		void* forceAndIndiceStreamBase = NULL;

		mDynamicsContext->getDataStreamBase(contactStreamBase, patchStreamBase, forceAndIndiceStreamBase);

		mLLContext->setNphaseFallbackImplementationContext(cpuNphaseImplementation);

		PxvNphaseImplementationContext* gpuNphaseImplementation = physxGpu->createGpuNphaseImplementationContext(*mLLContext, mGpuWranglerManagers, cpuNphaseImplementation, desc.gpuDynamicsConfig, contactStreamBase, patchStreamBase,
			forceAndIndiceStreamBase, getBoundsArray().getBounds(), &mSimpleIslandManager->getAccurateIslandSim(), mDynamicsContext, desc.gpuComputeVersion, mHeapMemoryAllocationManager, useGpuBroadphase);

		mSimulationControllerCallback = PX_NEW(PxgSimulationControllerCallback)(this);

		mSimulationController = physxGpu->createGpuSimulationController(mGpuWranglerManagers, mLLContext->getCudaContextManager(),
			mDynamicsContext, gpuNphaseImplementation, broadPhase, useGpuBroadphase, mSimulationControllerCallback, desc.gpuComputeVersion, mHeapMemoryAllocationManager,
			(desc.gpuDynamicsConfig.maxSoftBodyContacts > 0) ? desc.gpuDynamicsConfig.maxSoftBodyContacts : desc.gpuDynamicsConfig.maxDeformableVolumeContacts,
			(desc.gpuDynamicsConfig.maxFemClothContacts > 0) ? desc.gpuDynamicsConfig.maxFemClothContacts : desc.gpuDynamicsConfig.maxDeformableSurfaceContacts,
			desc.gpuDynamicsConfig.maxParticleContacts,
			desc.gpuDynamicsConfig.collisionStackSize, enableBodyAccelerations);

		mSimulationController->setBounds(mBoundsArray);
		mDynamicsContext->setSimulationController(mSimulationController);

		mLLContext->setNphaseImplementationContext(gpuNphaseImplementation);

		mLLContext->mContactStreamPool = &mDynamicsContext->getContactStreamPool();
		mLLContext->mPatchStreamPool = &mDynamicsContext->getPatchStreamPool();
		mLLContext->mForceAndIndiceStreamPool = &mDynamicsContext->getForceStreamPool();
		mLLContext->mFrictionPatchStreamPool = &mDynamicsContext->getFrictionPatchStreamPool();

		// PT: TODO: what's the difference between this allocator and "allocator" above?
		PxVirtualAllocator tAllocator(mHeapMemoryAllocationManager->mMappedMemoryAllocators, PxsHeapStats::eBROADPHASE);

		if (!useGpuBroadphase)
			// PT: TODO: we're using a CUDA allocator in the CPU broadphase, and a different allocator for the bounds array?
			mAABBManager = createAABBManagerCPU(desc, broadPhase, mBoundsArray, mContactDistance, tAllocator, contextID);
		else
			mAABBManager = createAABBManagerGPU(mGpuWranglerManagers, mLLContext->getCudaContextManager(), mHeapMemoryAllocationManager, desc, broadPhase, mBoundsArray, mContactDistance, tAllocator, contextID);
#endif
	}

	//Construct the bitmap of updated actors required as input to the broadphase update
	if(desc.limits.maxNbBodies)
	{
		// PT: TODO: revisit this. Why do we handle the added/removed and updated bitmaps entirely differently, in different places? And what is this weird formula here?
		mAABBManager->getChangedAABBMgActorHandleMap().resize((2*desc.limits.maxNbBodies + 256) & ~255);
	}

	//mLLContext->createTransformCache(mDynamicsContext->getAllocatorCallback());

	mLLContext->createTransformCache(*allocatorCallback);
	mLLContext->setContactDistance(mContactDistance);

	mCCDContext = PX_NEW(PxsCCDContext)(mLLContext, mDynamicsContext->getThresholdStream(), *mLLContext->getNphaseImplementationContext(), desc.ccdThreshold);
	
	setSolverBatchSize(desc.solverBatchSize);
	setSolverArticBatchSize(desc.solverArticulationBatchSize);
	mDynamicsContext->setFrictionOffsetThreshold(desc.frictionOffsetThreshold);
	mDynamicsContext->setCCDSeparationThreshold(desc.ccdMaxSeparation);
	mDynamicsContext->setCorrelationDistance(desc.frictionCorrelationDistance);

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mLLContext->setMeshContactMargin(0.01f * scale.length);
	mLLContext->setToleranceLength(scale.length);

	// the original descriptor uses 
	//    bounce iff impact velocity  > threshold
	// but LL use 
	//    bounce iff separation velocity < -threshold 
	// hence we negate here.

	mDynamicsContext->setBounceThreshold(-desc.bounceThresholdVelocity);

	mStaticAnchor = mStaticSimPool->construct(*this, mAnchorCore);

	mNPhaseCore = PX_NEW(NPhaseCore)(*this, desc);

	// Init dominance matrix
	{
		//init all dominance pairs such that:
		//if g1 == g2, then (1.0f, 1.0f) is returned
		//if g1 <  g2, then (0.0f, 1.0f) is returned
		//if g1 >  g2, then (1.0f, 0.0f) is returned

		PxU32 mask = ~PxU32(1);
		for (unsigned i = 0; i < PX_MAX_DOMINANCE_GROUP; ++i, mask <<= 1)
			mDominanceBitMatrix[i] = ~mask;
	}
		
//	DeterminismDebugger::begin();

	mWokeBodyListValid = true;
	mSleepBodyListValid = true;

	//load from desc:
	setLimits(desc.limits);

	// Create broad phase
	mBroadphaseManager.setBroadPhaseCallback(desc.broadPhaseCallback);

	setGravity(desc.gravity);

	setPCM(desc.flags & PxSceneFlag::eENABLE_PCM);

	setContactCache(!(desc.flags & PxSceneFlag::eDISABLE_CONTACT_CACHE));
	setSimulationEventCallback(desc.simulationEventCallback);
	setContactModifyCallback(desc.contactModifyCallback);
	setCCDContactModifyCallback(desc.ccdContactModifyCallback);
	if (desc.deformableVolumePostSolveCallback)
		setDeformableVolumeGpuPostSolveCallback(desc.deformableVolumePostSolveCallback);
	if (desc.deformableSurfacePostSolveCallback)
		setDeformableSurfaceGpuPostSolveCallback(desc.deformableSurfacePostSolveCallback);
	setCCDMaxPasses(desc.ccdMaxPasses);
	PX_ASSERT(mNPhaseCore); // refactor paranoia
	
	PX_ASSERT(	((desc.filterShaderData) && (desc.filterShaderDataSize > 0)) ||
				(!(desc.filterShaderData) && (desc.filterShaderDataSize == 0))	);
	if (desc.filterShaderData)
	{
		mFilterShaderData = PX_ALLOC(desc.filterShaderDataSize, sFilterShaderDataMemAllocId);
		PxMemCopy(mFilterShaderData, desc.filterShaderData, desc.filterShaderDataSize);
		mFilterShaderDataSize = desc.filterShaderDataSize;
		mFilterShaderDataCapacity = desc.filterShaderDataSize;
	}
	else
	{
		mFilterShaderData = NULL;
		mFilterShaderDataSize = 0;
		mFilterShaderDataCapacity = 0;
	}
	mFilterShader = desc.filterShader;
	mFilterCallback = desc.filterCallback;
}

void ev4sio_Sc::Scene::release()
{
	// TODO: PT: check virtual stuff

	mTimeStamp++;

	//collisionSpace.purgeAllPairs();

	//purgePairs();
	//releaseTagData();

	// We know release all the shapes before the collision space
	//collisionSpace.deleteAllShapes();

	//collisionSpace.release();

	//DeterminismDebugger::end();

	PX_FREE(mActiveKinematicsCopy);

	PX_DELETE(mNPhaseCore);

	PX_FREE(mFilterShaderData);

	if(mStaticAnchor)
		mStaticSimPool->destroy(mStaticAnchor);

	// Free object IDs and the deleted object id map
	postReportsCleanup();

	//before the task manager
	if (mLLContext)
	{
		if(mLLContext->getNphaseFallbackImplementationContext())
		{
			mLLContext->getNphaseFallbackImplementationContext()->destroy();
			mLLContext->setNphaseFallbackImplementationContext(NULL);
		}

		if(mLLContext->getNphaseImplementationContext())
		{
			mLLContext->getNphaseImplementationContext()->destroy();
			mLLContext->setNphaseImplementationContext(NULL);
		}
	}

	PX_DELETE(mSqBoundsManager);
	PX_DELETE(mBoundsArray);

	PX_DELETE(mSimStateDataPool);
	PX_DELETE(mStaticSimPool);
	PX_DELETE(mShapeSimPool);
	PX_DELETE(mBodySimPool);
	PX_DELETE(mLLArticulationRCPool);
#if PX_SUPPORT_GPU_PHYSX
	gpu_releasePools();
#endif
	mTriggerBufferExtraData->~TriggerBufferExtraData();
	PX_FREE(mTriggerBufferExtraData);

	PX_DELETE(mElementIDPool);
	PX_DELETE(mActorIDTracker);
	PX_DELETE(mConstraintIDTracker);
	PX_DELETE(mStats);

	ev4sio_Bp::BroadPhase* broadPhase = mAABBManager->getBroadPhase();
	mAABBManager->destroy();
	PX_RELEASE(broadPhase);

	PX_DELETE(mSimulationControllerCallback);
	PX_DELETE(mSimulationController);

	mDynamicsContext->destroy();

	PX_DELETE(mCCDContext);

	PX_DELETE(mSimpleIslandManager);

#if PX_SUPPORT_GPU_PHYSX
	gpu_release();
#endif

	PX_RELEASE(mTaskManager);
	PX_DELETE(mLLContext);

	// PT: TODO: revisit this
	mContactDistance->~PxFloatArrayPinned();
	PX_FREE(mContactDistance);

	PX_DELETE(mMemoryManager);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxSceneResidual ev4sio_Sc::Scene::getSolverResidual()	const
{
	if (!(getFlags() & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING))
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "Proper solver residual values can only be provided if the scene flag PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING is set");
	return mResidual;
}

void ev4sio_Sc::Scene::preAllocate(PxU32 nbStatics, PxU32 nbBodies, PxU32 nbStaticShapes, PxU32 nbDynamicShapes)
{
	// PT: TODO: this is only used for my addActors benchmark for now. Pre-allocate more arrays here.

	mActiveBodies.reserve(PxMax<PxU32>(64,nbBodies));

	mStaticSimPool->preAllocate(nbStatics);

	mBodySimPool->preAllocate(nbBodies);

	mShapeSimPool->preAllocate(nbStaticShapes + nbDynamicShapes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::addDirtyArticulationSim(ev4sio_Sc::ArticulationSim* artiSim)
{
	artiSim->setDirtyFlag(ArticulationSimDirtyFlag::eUPDATE);
	mDirtyArticulationSims.insert(artiSim);
}

void ev4sio_Sc::Scene::removeDirtyArticulationSim(ev4sio_Sc::ArticulationSim* artiSim)
{
	artiSim->setDirtyFlag(ArticulationSimDirtyFlag::eNONE);
	mDirtyArticulationSims.erase(artiSim);
}

void ev4sio_Sc::Scene::addToActiveList(ActorSim& actorSim)
{
	PX_ASSERT(actorSim.getActiveListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);

	ActorCore* appendedActorCore = &actorSim.getActorCore();

	if (actorSim.isDynamicRigid())
	{
		// Sort: kinematic before dynamic
		const PxU32 size = mActiveBodies.size();
		PxU32 incomingBodyActiveListIndex = size;							// PT: by default we append the incoming body at the end of the current array.

		BodySim& bodySim = static_cast<BodySim&>(actorSim);
		if (bodySim.isKinematic())											// PT: Except if incoming body is kinematic, in which case:
		{
			const PxU32 nbKinematics = mActiveKinematicBodyCount++;			// PT: - we increase their number
			if (nbKinematics != size)										// PT: - if there's at least one dynamic in the array...
			{
				PX_ASSERT(appendedActorCore != mActiveBodies[nbKinematics]);
				appendedActorCore = mActiveBodies[nbKinematics];			// PT: ...then we grab the first dynamic after the kinematics...
				appendedActorCore->getSim()->setActiveListIndex(size);		// PT: ...and we move that one back to the end of the array...

				mActiveBodies[nbKinematics] = static_cast<BodyCore*>(&actorSim.getActorCore());			// PT: ...while the incoming kine replaces the dynamic we moved out.
				incomingBodyActiveListIndex = nbKinematics;					// PT: ...thus the incoming kine's index is the prev #kines.
			}
		}
		
		// for active compound rigids add to separate array, so we dont have to traverse all active actors
		if (bodySim.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
		{
			PX_ASSERT(actorSim.getActiveCompoundListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);
			const PxU32 compoundIndex = mActiveCompoundBodies.size();
			mActiveCompoundBodies.pushBack(static_cast<BodyCore*>(appendedActorCore));
			actorSim.setActiveCompoundListIndex(compoundIndex);
		}

		actorSim.setActiveListIndex(incomingBodyActiveListIndex);			// PT: will be 'size' or 'nbKinematics', 'dynamicIndex'
		mActiveBodies.pushBack(static_cast<BodyCore*>(appendedActorCore));	// PT: will be the incoming object or the first dynamic we moved out.
	}
#if PX_SUPPORT_GPU_PHYSX
	else
		gpu_addToActiveList(actorSim, appendedActorCore);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void removeFromActiveCompoundBodyList(ev4sio_Sc::ActorSim& actorSim, PxArray<ev4sio_Sc::BodyCore*>& activeCompoundBodies)
{
	const PxU32 removedCompoundIndex = actorSim.getActiveCompoundListIndex();
	PX_ASSERT(removedCompoundIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
	actorSim.setActiveCompoundListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

	const PxU32 newCompoundSize = activeCompoundBodies.size() - 1;

	if(removedCompoundIndex != newCompoundSize)
	{
		ev4sio_Sc::BodyCore* lastBody = activeCompoundBodies[newCompoundSize];
		activeCompoundBodies[removedCompoundIndex] = lastBody;
		lastBody->getSim()->setActiveCompoundListIndex(removedCompoundIndex);
	}
	activeCompoundBodies.forceSize_Unsafe(newCompoundSize);
}

void ev4sio_Sc::Scene::removeFromActiveCompoundBodyList(BodySim& body)
{
	::removeFromActiveCompoundBodyList(body, mActiveCompoundBodies);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::removeFromActiveList(ActorSim& actorSim)
{
	PxU32 removedActiveIndex = actorSim.getActiveListIndex();
	PX_ASSERT(removedActiveIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
	actorSim.setActiveListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

	if (actorSim.isDynamicRigid())
	{
		PX_ASSERT(mActiveBodies[removedActiveIndex] == &actorSim.getActorCore());

		const PxU32 newSize = mActiveBodies.size() - 1;

		BodySim& bodySim = static_cast<BodySim&>(actorSim);

		// Sort: kinematic before dynamic,
		if (removedActiveIndex < mActiveKinematicBodyCount)	// PT: same as 'body.isKinematic()' but without accessing the Core data
		{
			PX_ASSERT(mActiveKinematicBodyCount);
			PX_ASSERT(bodySim.isKinematic());
			const PxU32 swapIndex = --mActiveKinematicBodyCount;
			if (newSize != swapIndex				// PT: equal if the array only contains kinematics
				&& removedActiveIndex < swapIndex)	// PT: i.e. "if we don't remove the last kinematic"
			{
				BodyCore* swapBody = mActiveBodies[swapIndex];
				swapBody->getSim()->setActiveListIndex(removedActiveIndex);
				mActiveBodies[removedActiveIndex] = swapBody;
				removedActiveIndex = swapIndex;
			}
		}

		// for active compound rigids add to separate array, so we dont have to traverse all active actors
		// A.B. TODO we should handle kinematic switch, no need to hold kinematic rigids in compound list
		if(bodySim.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
			::removeFromActiveCompoundBodyList(actorSim, mActiveCompoundBodies);

		if (removedActiveIndex != newSize)
		{
			ev4sio_Sc::BodyCore* lastBody = mActiveBodies[newSize];
			mActiveBodies[removedActiveIndex] = lastBody;
			lastBody->getSim()->setActiveListIndex(removedActiveIndex);
		}
		mActiveBodies.forceSize_Unsafe(newSize);
	}
#if PX_SUPPORT_GPU_PHYSX
	else
		gpu_removeFromActiveList(actorSim, removedActiveIndex);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::swapInActiveBodyList(BodySim& body)
{
	PX_ASSERT(!body.isStaticRigid() && !body.isDeformableSurface() && !body.isDeformableVolume() && !body.isParticleSystem());
	const PxU32 activeListIndex = body.getActiveListIndex();
	PX_ASSERT(activeListIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);

	PxU32 swapIndex;
	PxU32 newActiveKinematicBodyCount;
	if(activeListIndex < mActiveKinematicBodyCount)
	{
		// kinematic -> dynamic
		PX_ASSERT(!body.isKinematic());  // the corresponding flag gets switched before this call
		PX_ASSERT(mActiveKinematicBodyCount > 0);  // there has to be at least one kinematic body

		swapIndex = mActiveKinematicBodyCount - 1;
		newActiveKinematicBodyCount = swapIndex;
	}
	else
	{
		// dynamic -> kinematic
		PX_ASSERT(body.isKinematic());  // the corresponding flag gets switched before this call
		PX_ASSERT(mActiveKinematicBodyCount < mActiveBodies.size());  // there has to be at least one dynamic body

		swapIndex = mActiveKinematicBodyCount;
		newActiveKinematicBodyCount = swapIndex + 1;
	}

	BodyCore*& swapBodyRef = mActiveBodies[swapIndex];
	body.setActiveListIndex(swapIndex);
	BodyCore* swapBody = swapBodyRef;
	swapBodyRef = &body.getBodyCore();

	swapBody->getSim()->setActiveListIndex(activeListIndex);
	mActiveBodies[activeListIndex] = swapBody;

	mActiveKinematicBodyCount = newActiveKinematicBodyCount;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::registerInteraction(ElementSimInteraction* interaction, bool active)
{
	const InteractionType::Enum type = interaction->getType();
	const PxU32 sceneArrayIndex = mInteractions[type].size();
	interaction->setInteractionId(sceneArrayIndex);

	mInteractions[type].pushBack(interaction);
	if (active)
	{
		if (sceneArrayIndex > mActiveInteractionCount[type])
			swapInteractionArrayIndices(sceneArrayIndex, mActiveInteractionCount[type], type);
		mActiveInteractionCount[type]++;
	}

	mNPhaseCore->registerInteraction(interaction);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::unregisterInteraction(ElementSimInteraction* interaction)
{
	const InteractionType::Enum type = interaction->getType();
	const PxU32 sceneArrayIndex = interaction->getInteractionId();
	PX_ASSERT(sceneArrayIndex != PX_INVALID_INTERACTION_SCENE_ID);
	if (sceneArrayIndex == PX_INVALID_INTERACTION_SCENE_ID)
	{
		// vreutskyy: Defensive coding added for OM-117470. In theory this should not be needed, as we
		// shouldn't be able to unregister twice. We could/should remove this eventually.
		// ### DEFENSIVE
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Unexpectedly unregistered an interaction that does not have a valid interaction ID.");
		return;
	}
	mInteractions[type].replaceWithLast(sceneArrayIndex);
	interaction->setInteractionId(PX_INVALID_INTERACTION_SCENE_ID);
	if (sceneArrayIndex<mInteractions[type].size()) // The removed interaction was the last one, do not reset its sceneArrayIndex
		mInteractions[type][sceneArrayIndex]->setInteractionId(sceneArrayIndex);
	if (sceneArrayIndex<mActiveInteractionCount[type])
	{
		mActiveInteractionCount[type]--;
		if (mActiveInteractionCount[type]<mInteractions[type].size())
			swapInteractionArrayIndices(sceneArrayIndex, mActiveInteractionCount[type], type);
	}

	mNPhaseCore->unregisterInteraction(interaction);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::swapInteractionArrayIndices(PxU32 id1, PxU32 id2, InteractionType::Enum type)
{
	PxArray<ElementSimInteraction*>& interArray = mInteractions[type];
	ElementSimInteraction* interaction1 = interArray[id1];
	ElementSimInteraction* interaction2 = interArray[id2];
	interArray[id1] = interaction2;
	interArray[id2] = interaction1;
	interaction1->setInteractionId(id2);
	interaction2->setInteractionId(id1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::notifyInteractionActivated(Interaction* interaction)
{
	PX_ASSERT((interaction->getType() == InteractionType::eOVERLAP) || (interaction->getType() == InteractionType::eTRIGGER));
	PX_ASSERT(interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_ASSERT(interaction->getInteractionId() != PX_INVALID_INTERACTION_SCENE_ID);

	const InteractionType::Enum type = interaction->getType();

	PX_ASSERT(interaction->getInteractionId() >= mActiveInteractionCount[type]);
	
	if (mActiveInteractionCount[type] < mInteractions[type].size())
		swapInteractionArrayIndices(mActiveInteractionCount[type], interaction->getInteractionId(), type);
	mActiveInteractionCount[type]++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::notifyInteractionDeactivated(Interaction* interaction)
{
	PX_ASSERT((interaction->getType() == InteractionType::eOVERLAP) || (interaction->getType() == InteractionType::eTRIGGER));
	PX_ASSERT(!interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_ASSERT(interaction->getInteractionId() != PX_INVALID_INTERACTION_SCENE_ID);

	const InteractionType::Enum type = interaction->getType();
	PX_ASSERT(interaction->getInteractionId() < mActiveInteractionCount[type]);

	if (mActiveInteractionCount[type] > 1)
		swapInteractionArrayIndices(mActiveInteractionCount[type]-1, interaction->getInteractionId(), type);
	mActiveInteractionCount[type]--;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void** ev4sio_Sc::Scene::allocatePointerBlock(PxU32 size)
{
	PX_ASSERT(size>32 || size == 32 || size == 16 || size == 8);
	void* ptr;
	if(size==8)
		ptr = mPointerBlock8Pool.construct();
	else if(size == 16)
		ptr = mPointerBlock16Pool.construct();
	else if(size == 32)
		ptr = mPointerBlock32Pool.construct();
	else
		ptr = PX_ALLOC(size * sizeof(void*), "void*");

	return reinterpret_cast<void**>(ptr);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::deallocatePointerBlock(void** block, PxU32 size)
{
	PX_ASSERT(size>32 || size == 32 || size == 16 || size == 8);
	if(size==8)
		mPointerBlock8Pool.destroy(reinterpret_cast<PointerBlock8*>(block));
	else if(size == 16)
		mPointerBlock16Pool.destroy(reinterpret_cast<PointerBlock16*>(block));
	else if(size == 32)
		mPointerBlock32Pool.destroy(reinterpret_cast<PointerBlock32*>(block));
	else
		PX_FREE(block);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::setFilterShaderData(const void* data, PxU32 dataSize)
{
	PX_UNUSED(sFilterShaderDataMemAllocId);

	if (data)
	{
		PX_ASSERT(dataSize > 0);

		void* buffer;

		if (dataSize <= mFilterShaderDataCapacity)
			buffer = mFilterShaderData;
		else
		{
			buffer = PX_ALLOC(dataSize, sFilterShaderDataMemAllocId);
			if (buffer)
			{
				mFilterShaderDataCapacity = dataSize;
				PX_FREE(mFilterShaderData);
			}
			else
			{
				outputError<PxErrorCode::eOUT_OF_MEMORY>(__LINE__, "Failed to allocate memory for filter shader data!");
				return;
			}
		}

		PxMemCopy(buffer, data, dataSize);
		mFilterShaderData = buffer;
		mFilterShaderDataSize = dataSize;
	}
	else
	{
		PX_ASSERT(dataSize == 0);

		PX_FREE(mFilterShaderData);
		mFilterShaderDataSize = 0;
		mFilterShaderDataCapacity = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//stepSetup is called in solve, but not collide
void ev4sio_Sc::Scene::stepSetupSolve(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.stepSetupSolve", mContextId);

	kinematicsSetup(continuation);
}

void ev4sio_Sc::Scene::advance(PxReal timeStep, PxBaseTask* continuation)
{
	if(timeStep != 0.0f)
	{
		setElapsedTime(timeStep);

		mAdvanceStep.setContinuation(continuation);

		stepSetupSolve(&mAdvanceStep);		
		
		mAdvanceStep.removeReference();
	}
}

void ev4sio_Sc::Scene::collide(PxReal timeStep, PxBaseTask* continuation)
{
	mDt = timeStep;

	stepSetupCollide(continuation);

	mLLContext->beginUpdate();

	mCollideStep.setContinuation(continuation);
	mCollideStep.removeReference();
}

void ev4sio_Sc::Scene::endSimulation()
{
	// Handle user contact filtering
	// Note: Do this before the contact callbacks get fired since the filter callback might
	//       trigger contact reports (touch lost due to re-filtering)

	mBroadphaseManager.prepareOutOfBoundsCallbacks(mAABBManager);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	mNPhaseCore->fireCustomFilteringCallbacks(outputs);

	mNPhaseCore->preparePersistentContactEventListForNextFrame();

	mSimulationController->releaseDeferredArticulationIds();

#if PX_SUPPORT_GPU_PHYSX
	mSimulationController->releaseDeferredSoftBodyIds();

	mSimulationController->releaseDeferredFEMClothIds();

	mSimulationController->releaseDeferredParticleSystemIds();

	mAABBManager->releaseDeferredAggregateIds();
#endif

	// End step / update time stamps
	{
		mTimeStamp++;
	//  INVALID_SLEEP_COUNTER is 0xffffffff. Therefore the last bit is masked. Look at Body::isForcedToSleep() for example.
	//	if(timeStamp==PX_INVALID_U32)	timeStamp = 0;	// Reserve INVALID_ID for something else
		mTimeStamp &= 0x7fffffff;

		mReportShapePairTimeStamp++;  // to make sure that deleted shapes/actors after fetchResults() create new report pairs
	}

	PxcDisplayContactCacheStats();
}

void ev4sio_Sc::Scene::flush(bool sendPendingReports)
{
	if (sendPendingReports)
	{
		fireQueuedContactCallbacks();
		mNPhaseCore->clearContactReportStream();
		mNPhaseCore->clearContactReportActorPairs(true);

		fireTriggerCallbacks();
	}
	else
	{
		mNPhaseCore->clearContactReportActorPairs(true);  // To clear the actor pair set
	}
	postReportsCleanup();
	mNPhaseCore->freeContactReportStreamMemory();

	mTriggerBufferAPI.reset();
	mTriggerBufferExtraData->reset();

	mBrokenConstraints.reset();

	mBroadphaseManager.flush(mAABBManager);

	clearSleepWakeBodies();  //!!! If we send out these reports on flush then this would not be necessary

	mActorIDTracker->reset();
	mElementIDPool->reset();

	processLostTouchPairs();  // Processes the lost touch bodies
	PX_ASSERT(mLostTouchPairs.size() == 0);
	mLostTouchPairs.reset();
	// Does not seem worth deleting the bitmap for the lost touch pair list

	mActiveBodies.shrink();

	for(PxU32 i=0; i < InteractionType::eTRACKED_IN_SCENE_COUNT; i++)
	{
		mInteractions[i].shrink();
	}

	//!!! TODO: look into retrieving memory from the NPhaseCore & Broadphase class (all the pools in there etc.)

	mLLContext->getNpMemBlockPool().releaseUnusedBlocks();
}

// User callbacks

void ev4sio_Sc::Scene::setSimulationEventCallback(PxSimulationEventCallback* callback)
{
	if(!mSimulationEventCallback && callback)
	{
		// if there was no callback before, the sleeping bodies have to be prepared for potential notification events (no shortcut possible anymore)
		BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
		for (PxU32 i = 0; i < mSleepBodies.size(); i++)
		{
			sleepingBodies[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		}

#if PX_SUPPORT_GPU_PHYSX
		gpu_setSimulationEventCallback(callback);
#endif
	}

	mSimulationEventCallback = callback;
}

PxSimulationEventCallback* ev4sio_Sc::Scene::getSimulationEventCallback() const
{
	return mSimulationEventCallback;
}

void ev4sio_Sc::Scene::removeBody(BodySim& body)	//this also notifies any connected joints!
{
	BodyCore& core = body.getBodyCore();

	// Remove from sleepBodies array
	mSleepBodies.erase(&core);
	PX_ASSERT(!mSleepBodies.contains(&core));

	// Remove from wokeBodies array
	mWokeBodies.erase(&core);
	PX_ASSERT(!mWokeBodies.contains(&core));

	if (body.isActive() && (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW))
		removeFromPosePreviewList(body);
	else
		PX_ASSERT(!isInPosePreviewList(body));

	markReleasedBodyIDForLostTouch(body.getActorID());
}

void ev4sio_Sc::Scene::addConstraint(ConstraintCore& constraint, RigidCore* body0, RigidCore* body1)
{
	ConstraintSim* sim = mConstraintSimPool.construct(constraint, body0, body1, *this);

	addConstraintToMap(constraint, body0, body1);

	mConstraints.insert(&constraint);

	getSimulationController()->addJoint(sim->getLowLevelConstraint());
}

void ev4sio_Sc::Scene::removeConstraint(ConstraintCore& constraint)
{
	ConstraintSim* cSim = constraint.getSim();

	if (cSim)
	{
		removeConstraintFromMap(*cSim->getInteraction());

		mConstraintSimPool.destroy(cSim);
	}

	mConstraints.erase(&constraint);
}

void ev4sio_Sc::Scene::addConstraintToMap(ConstraintCore& constraint, RigidCore* body0, RigidCore* body1)
{
	PxNodeIndex nodeIndex0, nodeIndex1;

	ActorSim* sim0 = NULL;
	ActorSim* sim1 = NULL;

	if (body0)
	{
		sim0 = body0->getSim();
		nodeIndex0 = sim0->getNodeIndex();
	}
	if (body1)
	{
		sim1 = body1->getSim();
		nodeIndex1 = sim1->getNodeIndex();
	}

	if (nodeIndex1 < nodeIndex0)
		PxSwap(sim0, sim1);

	mConstraintMap.insert(PxPair<const ev4sio_Sc::ActorSim*, const ev4sio_Sc::ActorSim*>(sim0, sim1), &constraint);
}

void ev4sio_Sc::Scene::removeConstraintFromMap(const ConstraintInteraction& interaction)
{
	PxNodeIndex nodeIndex0, nodeIndex1;

	ev4sio_Sc::ActorSim* bSim = &interaction.getActorSim0();
	ev4sio_Sc::ActorSim* bSim1 = &interaction.getActorSim1();

	if (bSim)
		nodeIndex0 = bSim->getNodeIndex();
	if (bSim1)
		nodeIndex1 = bSim1->getNodeIndex();

	if (nodeIndex1 < nodeIndex0)
		PxSwap(bSim, bSim1);

	mConstraintMap.erase(PxPair<const ev4sio_Sc::ActorSim*, const ev4sio_Sc::ActorSim*>(bSim, bSim1));
}

void ev4sio_Sc::Scene::addArticulation(ArticulationCore& articulation, BodyCore& root)
{
	ArticulationSim* sim = PX_NEW(ArticulationSim)(articulation, *this, root);

	if (sim && (sim->getLowLevelArticulation() == NULL))
	{
		PX_DELETE(sim);
		return;
	}
	mArticulations.insert(&articulation);

	addDirtyArticulationSim(sim);
}

void ev4sio_Sc::Scene::removeArticulation(ArticulationCore& articulation)
{
	ArticulationSim* a = articulation.getSim();

	ev4sio_Sc::ArticulationSimDirtyFlags dirtyFlags = a->getDirtyFlag();
	const bool isDirty = (dirtyFlags & ev4sio_Sc::ArticulationSimDirtyFlag::eUPDATE);
	if(isDirty)
		removeDirtyArticulationSim(a);

	PX_DELETE(a);
	mArticulations.erase(&articulation);
}

void ev4sio_Sc::Scene::addArticulationJoint(ArticulationJointCore& joint, BodyCore& parent, BodyCore& child)
{
	ArticulationJointSim* sim = mArticulationJointSimPool.construct(joint, *parent.getSim(), *child.getSim());
	PX_UNUSED(sim);
}

void ev4sio_Sc::Scene::removeArticulationJoint(ArticulationJointCore& joint)
{
	ArticulationJointSim* sim = joint.getSim();
	mArticulationJointSimPool.destroy(sim);
}

void ev4sio_Sc::Scene::addArticulationTendon(ArticulationSpatialTendonCore& tendon)
{
	ArticulationSpatialTendonSim* sim = PX_NEW(ArticulationSpatialTendonSim)(tendon, *this);
	PX_UNUSED(sim);
}

void ev4sio_Sc::Scene::removeArticulationTendon(ArticulationSpatialTendonCore& tendon)
{
	ArticulationSpatialTendonSim* sim = tendon.getSim();
	PX_DELETE(sim);
}

void ev4sio_Sc::Scene::addArticulationTendon(ArticulationFixedTendonCore& tendon)
{
	ArticulationFixedTendonSim* sim = PX_NEW(ArticulationFixedTendonSim)(tendon, *this);

	PX_UNUSED(sim);
}

void ev4sio_Sc::Scene::removeArticulationTendon(ArticulationFixedTendonCore& tendon)
{
	ArticulationFixedTendonSim* sim = tendon.getSim();
	PX_DELETE(sim);
}

void ev4sio_Sc::Scene::addArticulationMimicJoint(ArticulationMimicJointCore& mimicJoint)
{
	//This might look like a forgotten allocation but it really isn't.
	//ArticulationMimicJointSim constructor does all the work here to make sure that
	//mimicJoint ends up maintaining a reference to sim.
	ArticulationMimicJointSim* sim = PX_NEW(ArticulationMimicJointSim)(mimicJoint, *this);
	PX_UNUSED(sim);
}

void ev4sio_Sc::Scene::removeArticulationMimicJoint(ArticulationMimicJointCore& mimicJoint)
{
	ArticulationMimicJointSim* sim = mimicJoint.getSim();
	PX_DELETE(sim);
}

void ev4sio_Sc::Scene::addArticulationSimControl(ev4sio_Sc::ArticulationCore& core)
{
	ev4sio_Sc::ArticulationSim* sim = core.getSim();
	if (sim)
		mSimulationController->addArticulation(sim->getLowLevelArticulation(), sim->getIslandNodeIndex());
}

void ev4sio_Sc::Scene::removeArticulationSimControl(ev4sio_Sc::ArticulationCore& core)
{
	ev4sio_Sc::ArticulationSim* sim = core.getSim();
	if (sim)
		mSimulationController->releaseArticulation(sim->getLowLevelArticulation(), sim->getIslandNodeIndex());
}

void* ev4sio_Sc::Scene::allocateConstraintBlock(PxU32 size)
{
	if(size<=128)
		return mMemBlock128Pool.construct();
	else if(size<=256)
		return mMemBlock256Pool.construct();
	else  if(size<=384)
		return mMemBlock384Pool.construct();
	else  if(size<=512)
		return mMemBlock512Pool.construct();
	else
		return PX_ALLOC(size, "ConstraintBlock");
}

void ev4sio_Sc::Scene::deallocateConstraintBlock(void* ptr, PxU32 size)
{
	if(size<=128)
		mMemBlock128Pool.destroy(reinterpret_cast<MemBlock128*>(ptr));
	else if(size<=256)
		mMemBlock256Pool.destroy(reinterpret_cast<MemBlock256*>(ptr));
	else  if(size<=384)
		mMemBlock384Pool.destroy(reinterpret_cast<MemBlock384*>(ptr));
	else  if(size<=512)
		mMemBlock512Pool.destroy(reinterpret_cast<MemBlock512*>(ptr));
	else
		PX_FREE(ptr);
}

void ev4sio_Sc::Scene::postReportsCleanup()
{
	mElementIDPool->processPendingReleases();
	mElementIDPool->clearDeletedIDMap();

	mActorIDTracker->processPendingReleases();
	mActorIDTracker->clearDeletedIDMap();

	mConstraintIDTracker->processPendingReleases();
	mConstraintIDTracker->clearDeletedIDMap();

#if PX_SUPPORT_GPU_PHYSX
	// AD: if we use either GPU BP or GPU dynamics.
	if (mHeapMemoryAllocationManager)
	{
		mHeapMemoryAllocationManager->flushDeferredDeallocs();		
	}
#endif
}

PX_COMPILE_TIME_ASSERT(sizeof(PxTransform32)==sizeof(PxsCachedTransform));

// PT: TODO: move this out of Sc? this is only called by Np
void ev4sio_Sc::Scene::syncSceneQueryBounds(SqBoundsSync& sync, SqRefFinder& finder)
{
	const PxsTransformCache& cache = mLLContext->getTransformCache();

	mSqBoundsManager->syncBounds(sync, finder, mBoundsArray->begin(), reinterpret_cast<const PxTransform32*>(cache.getTransforms()), mContextId, mDirtyShapeSimMap);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ev4sio_Sc::Scene::resizeReleasedBodyIDMaps(PxU32 maxActors, PxU32 numActors)
{ 
	mLostTouchPairsDeletedBodyIDs.resize(maxActors);
	mActorIDTracker->resizeDeletedIDMap(maxActors,numActors); 
	mElementIDPool->resizeDeletedIDMap(maxActors,numActors);
}

void ev4sio_Sc::Scene::finalizeContactStreamAndCreateHeader(PxContactPairHeader& header, const ActorPairReport& aPair, ContactStreamManager& cs, PxU32 removedShapeTestMask)
{
	PxU8* stream = mNPhaseCore->getContactReportPairData(cs.bufferIndex);
	PxU32 streamManagerFlag = cs.getFlags();
	ContactShapePair* contactPairs = cs.getShapePairs(stream);
	const PxU32 nbShapePairs = cs.currentPairCount;
	PX_ASSERT(nbShapePairs > 0);

	if (streamManagerFlag & removedShapeTestMask)
	{
		// At least one shape of this actor pair has been deleted. Need to traverse the contact buffer,
		// find the pairs which contain deleted shapes and set the flags accordingly.

		ContactStreamManager::convertDeletedShapesInContactStream(contactPairs, nbShapePairs, getElementIDPool());
	}
	PX_ASSERT(contactPairs);

	ObjectIDTracker& ActorIDTracker = getActorIDTracker();
	header.actors[0] = aPair.getPxActorA();
	header.actors[1] = aPair.getPxActorB();
	PxU16 headerFlags = 0;
	if (ActorIDTracker.isDeletedID(aPair.getActorAID()))
		headerFlags |= PxContactPairHeaderFlag::eREMOVED_ACTOR_0;
	if (ActorIDTracker.isDeletedID(aPair.getActorBID()))
		headerFlags |= PxContactPairHeaderFlag::eREMOVED_ACTOR_1;
	header.flags = PxContactPairHeaderFlags(headerFlags);
	header.pairs = reinterpret_cast<PxContactPair*>(contactPairs);
	header.nbPairs = nbShapePairs;

	PxU16 extraDataSize = cs.extraDataSize;
	if (!extraDataSize)
		header.extraDataStream = NULL;
	else
	{
		PX_ASSERT(extraDataSize >= sizeof(ContactStreamHeader));
		extraDataSize -= sizeof(ContactStreamHeader);
		header.extraDataStream = stream + sizeof(ContactStreamHeader);

		if (streamManagerFlag & ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY)
		{
			PX_ASSERT(!(headerFlags & PxTo16(PxContactPairHeaderFlag::eREMOVED_ACTOR_0 | PxContactPairHeaderFlag::eREMOVED_ACTOR_1)));
			cs.setContactReportPostSolverVelocity(stream, aPair.getActorA(), aPair.getActorB());
		}
	}
	header.extraDataStreamSize = extraDataSize;
}

const PxArray<PxContactPairHeader>& ev4sio_Sc::Scene::getQueuedContactPairHeaders()
{
	const PxU32 removedShapeTestMask = PxU32(ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES);

	ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
	PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
	mQueuedContactPairHeaders.reserve(nbActorPairs);
	mQueuedContactPairHeaders.clear();

	for (PxU32 i = 0; i < nbActorPairs; i++)
	{
		if (i < (nbActorPairs - 1))
			PxPrefetchLine(actorPairs[i + 1]);

		ActorPairReport* aPair = actorPairs[i];
		ContactStreamManager& cs = aPair->getContactStreamManager();
		if (cs.getFlags() & ContactStreamManagerFlag::eINVALID_STREAM)
			continue;

		if (i + 1 < nbActorPairs)
			PxPrefetch(&(actorPairs[i + 1]->getContactStreamManager()));

		PxContactPairHeader &pairHeader = mQueuedContactPairHeaders.insert();
		finalizeContactStreamAndCreateHeader(pairHeader, *aPair, cs, removedShapeTestMask);

		cs.maxPairCount = cs.currentPairCount;
		cs.setMaxExtraDataSize(cs.extraDataSize);
	}

	return mQueuedContactPairHeaders;
}

/*
Threading: called in the context of the user thread, but only after the physics thread has finished its run
*/
void ev4sio_Sc::Scene::fireQueuedContactCallbacks()
{
	if(mSimulationEventCallback)
	{
		const PxU32 removedShapeTestMask = PxU32(ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES);

		ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
		PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
		for(PxU32 i=0; i < nbActorPairs; i++)
		{
			if (i < (nbActorPairs - 1))
				PxPrefetchLine(actorPairs[i+1]);

			ActorPairReport* aPair = actorPairs[i];
			ContactStreamManager* cs = &aPair->getContactStreamManager();
			if (cs == NULL || cs->getFlags() & ContactStreamManagerFlag::eINVALID_STREAM)
				continue;
			
			if (i + 1 < nbActorPairs)
				PxPrefetch(&(actorPairs[i+1]->getContactStreamManager()));

			PxContactPairHeader pairHeader;
			finalizeContactStreamAndCreateHeader(pairHeader, *aPair, *cs, removedShapeTestMask);

			{
				PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onContact", mContextId);
				mSimulationEventCallback->onContact(pairHeader, pairHeader.pairs, pairHeader.nbPairs);
			}

			// estimates for next frame
			cs->maxPairCount = cs->currentPairCount;
			cs->setMaxExtraDataSize(cs->extraDataSize);
		}
	}
}

PX_FORCE_INLINE void markDeletedShapes(ev4sio_Sc::ObjectIDTracker& idTracker, ev4sio_Sc::TriggerPairExtraData& tped, PxTriggerPair& pair)
{
	PxTriggerPairFlags::InternalType flags = 0;
	if (idTracker.isDeletedID(tped.shape0ID))
		flags |= PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER;
	if (idTracker.isDeletedID(tped.shape1ID))
		flags |= PxTriggerPairFlag::eREMOVED_SHAPE_OTHER;

	pair.flags = PxTriggerPairFlags(flags);
}

void ev4sio_Sc::Scene::fireTriggerCallbacks()
{
	// triggers
	const PxU32 nbTriggerPairs = mTriggerBufferAPI.size();
	PX_ASSERT(nbTriggerPairs == mTriggerBufferExtraData->size());
	if(nbTriggerPairs) 
	{
		// cases to take into account:
		// - no simulation/trigger shape has been removed -> no need to test shape references for removed shapes
		// - simulation/trigger shapes have been removed  -> test the events that have 
		//   a marker for removed shapes set
		//
		const bool hasRemovedShapes = mElementIDPool->getDeletedIDCount() > 0;

		if(mSimulationEventCallback)
		{
			if (hasRemovedShapes)
			{
				for(PxU32 i = 0; i < nbTriggerPairs; i++)
				{
					PxTriggerPair& triggerPair = mTriggerBufferAPI[i];

					if ((PxTriggerPairFlags::InternalType(triggerPair.flags) & TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES))
						markDeletedShapes(*mElementIDPool, (*mTriggerBufferExtraData)[i], triggerPair);
				}
			}

			{
				PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onTrigger", mContextId);
				mSimulationEventCallback->onTrigger(mTriggerBufferAPI.begin(), nbTriggerPairs);
			}
		}
	}

	// PT: clear the buffer **even when there's no simulationEventCallback**.
	mTriggerBufferAPI.clear();
	mTriggerBufferExtraData->clear();
}

/*
Threading: called in the context of the user thread, but only after the physics thread has finished its run
*/
void ev4sio_Sc::Scene::fireCallbacksPostSync()
{
	//
	// Fire sleep & woken callbacks
	//

	// A body should be either in the sleep or the woken list. If it is in both, remove it from the list it was
	// least recently added to.

	if(!mSleepBodyListValid)
		cleanUpSleepBodies();

	if(!mWokeBodyListValid)
		cleanUpWokenBodies();

#if PX_SUPPORT_GPU_PHYSX
	const PxU32 maxGpuSizeNeeded = gpu_cleanUpSleepAndWokenBodies();
#endif

	if(mSimulationEventCallback || mOnSleepingStateChanged)
	{
		// allocate temporary data
		const PxU32 nbSleep = mSleepBodies.size();
		const PxU32 nbWoken = mWokeBodies.size();
#if PX_SUPPORT_GPU_PHYSX
		const PxU32 arrSize = PxMax(PxMax(nbSleep, nbWoken), maxGpuSizeNeeded);
#else
		const PxU32 arrSize = PxMax(nbSleep, nbWoken);
#endif
		PxActor** actors = arrSize ? reinterpret_cast<PxActor**>(PX_ALLOC(arrSize*sizeof(PxActor*), "PxActor*")) : NULL;
		if(actors)
		{
			if(nbSleep)
			{
				PxU32 destSlot = 0;
				BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
				for(PxU32 i=0; i<nbSleep; i++)
				{
					BodyCore* body = sleepingBodies[i];
					if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(*static_cast<PxRigidDynamic*>(body->getPxActor()), true);
				}

				if(destSlot && mSimulationEventCallback)
				{
					PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onSleep", mContextId);
					mSimulationEventCallback->onSleep(actors, destSlot);
				}

				//if (PX_DBG_IS_CONNECTED())
				//{
				//	for (PxU32 i = 0; i < nbSleep; ++i)
				//	{
				//		BodyCore* body = mSleepBodies[i];
				//		PX_ASSERT(body->getActorType() == PxActorType::eRIGID_DYNAMIC);
				//	}
				//}
			}

#if PX_SUPPORT_GPU_PHYSX
			gpu_fireOnSleepCallback(actors);
#endif

			// do the same thing for bodies that have just woken up

			if(nbWoken)
			{
				PxU32 destSlot = 0;
				BodyCore* const* wokenBodies = mWokeBodies.getEntries();
				for(PxU32 i=0; i<nbWoken; i++)
				{
					BodyCore* body = wokenBodies[i];
					if(body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
					if (mOnSleepingStateChanged)
						mOnSleepingStateChanged(*static_cast<PxRigidDynamic*>(body->getPxActor()), false);
				}

				if(destSlot && mSimulationEventCallback)
				{
					PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onWake", mContextId);
					mSimulationEventCallback->onWake(actors, destSlot);
				}
			}

#if PX_SUPPORT_GPU_PHYSX
			gpu_fireOnWakeCallback(actors);
#endif
			PX_FREE(actors);
		}
	}

	clearSleepWakeBodies();
}

void ev4sio_Sc::Scene::postCallbacksPreSync()
{
	PX_PROFILE_ZONE("Sim.postCallbackPreSync", mContextId);
	// clear contact stream data
	mNPhaseCore->clearContactReportStream();
	mNPhaseCore->clearContactReportActorPairs(false);

	postCallbacksPreSyncKinematics();

	releaseConstraints(true); //release constraint blocks at the end of the frame, so user can retrieve the blocks
}

void ev4sio_Sc::Scene::getStats(PxSimulationStatistics& s) const
{
	mStats->readOut(s, mLLContext->getSimStats());
	s.nbStaticBodies = mNbRigidStatics;
	s.nbDynamicBodies = mNbRigidDynamics;
	s.nbKinematicBodies = mNbRigidKinematic;
	s.nbArticulations = mArticulations.size(); 

	s.nbAggregates = mAABBManager->getNbActiveAggregates();
	for(PxU32 i=0; i<PxGeometryType::eGEOMETRY_COUNT; i++)
		s.nbShapes[i] = mNbGeometries[i];

#if PX_SUPPORT_GPU_PHYSX
	if (mHeapMemoryAllocationManager)
	{
		s.gpuMemHeap = mHeapMemoryAllocationManager->getDeviceMemorySize();

		const PxsHeapStats& deviceHeapStats = mHeapMemoryAllocationManager->getDeviceHeapStats();
		s.gpuMemHeapBroadPhase = deviceHeapStats.stats[PxsHeapStats::eBROADPHASE];
		s.gpuMemHeapNarrowPhase = deviceHeapStats.stats[PxsHeapStats::eNARROWPHASE];
		s.gpuMemHeapSolver = deviceHeapStats.stats[PxsHeapStats::eSOLVER];
		s.gpuMemHeapArticulation = deviceHeapStats.stats[PxsHeapStats::eARTICULATION];
		s.gpuMemHeapSimulation = deviceHeapStats.stats[PxsHeapStats::eSIMULATION];
		s.gpuMemHeapSimulationArticulation = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_ARTICULATION];
		s.gpuMemHeapSimulationParticles = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_PARTICLES];
		s.gpuMemHeapSimulationDeformableSurface = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_FEMCLOTH];
		s.gpuMemHeapSimulationDeformableVolume = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_SOFTBODY];
		s.gpuMemHeapSimulationSoftBody = deviceHeapStats.stats[PxsHeapStats::eSIMULATION_SOFTBODY]; //deprecated
		s.gpuMemHeapParticles = deviceHeapStats.stats[PxsHeapStats::eSHARED_PARTICLES];
		s.gpuMemHeapDeformableSurfaces = deviceHeapStats.stats[PxsHeapStats::eSHARED_FEMCLOTH];
		s.gpuMemHeapDeformableVolumes = deviceHeapStats.stats[PxsHeapStats::eSHARED_SOFTBODY];
		s.gpuMemHeapSoftBodies = deviceHeapStats.stats[PxsHeapStats::eSHARED_SOFTBODY]; //deprecated
		s.gpuMemHeapOther = deviceHeapStats.stats[PxsHeapStats::eOTHER];
	}
	else
#endif
	{
		s.gpuMemHeap = 0;
		s.gpuMemParticles = 0;
		s.gpuMemDeformableSurfaces = 0;
		s.gpuMemDeformableVolumes = 0;
		s.gpuMemSoftBodies = 0; // deprecated
		s.gpuMemHeapBroadPhase = 0;
		s.gpuMemHeapNarrowPhase = 0;
		s.gpuMemHeapSolver = 0;
		s.gpuMemHeapArticulation = 0;
		s.gpuMemHeapSimulation = 0;
		s.gpuMemHeapSimulationArticulation = 0;
		s.gpuMemHeapSimulationParticles = 0;
		s.gpuMemHeapSimulationDeformableSurface = 0;
		s.gpuMemHeapSimulationDeformableVolume = 0;
		s.gpuMemHeapSimulationSoftBody = 0; // deprecated
		s.gpuMemHeapParticles = 0;
		s.gpuMemHeapSoftBodies = 0; // deprecated
		s.gpuMemHeapDeformableSurfaces = 0;
		s.gpuMemHeapOther = 0;
	}
}

void ev4sio_Sc::Scene::addShapes(NpShape *const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& bodySim, PxBounds3* outBounds)
{
	const PxNodeIndex nodeIndex = bodySim.getNodeIndex();
	
	PxvNphaseImplementationContext* context = mLLContext->getNphaseImplementationContext();

	for(PxU32 i=0;i<nbShapes;i++)
	{
		// PT: TODO: drop the offsets and let me include NpShape.h from here! This is just NpShape::getCore()....
		ShapeCore& sc = *reinterpret_cast<ShapeCore*>(size_t(shapes[i])+ptrOffset);

		//PxBounds3* target = uninflatedBounds ? uninflatedBounds + i : uninflatedBounds;
		//mShapeSimPool->construct(sim, sc, llBody, target);

		ShapeSim* shapeSim = mShapeSimPool->construct(bodySim, sc);
		mNbGeometries[sc.getGeometryType()]++;

		mSimulationController->addPxgShape(shapeSim, shapeSim->getPxsShapeCore(), shapeSim->getActorNodeIndex(), shapeSim->getElementID());

		if (outBounds)
			outBounds[i] = mBoundsArray->getBounds(shapeSim->getElementID());
		
		//I register the shape if its either not an articulation link or if the nodeIndex has already been
		//assigned. On insertion, the articulation will not have this nodeIndex correctly assigned at this stage
		if (bodySim.getActorType() != PxActorType::eARTICULATION_LINK || !nodeIndex.isStaticBody())
			context->registerShape(nodeIndex, sc.getCore(), shapeSim->getElementID(), bodySim.getPxActor());
	}
}

void ev4sio_Sc::Scene::removeShapes(ev4sio_Sc::RigidSim& sim, PxInlineArray<ev4sio_Sc::ShapeSim*, 64>& shapesBuffer , PxInlineArray<const ev4sio_Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	PxU32 nbElems = sim.getNbElements();
	ev4sio_Sc::ElementSim** elems = sim.getElements();
	while (nbElems--)
	{
		ev4sio_Sc::ShapeSim* s = static_cast<ev4sio_Sc::ShapeSim*>(*elems++);
		// can do two 2x the allocs in the worst case, but actors with >64 shapes are not common
		shapesBuffer.pushBack(s);
		removedShapes.pushBack(&s->getCore());
	}

	for(PxU32 i=0;i<shapesBuffer.size();i++)
		removeShape_(*shapesBuffer[i], wakeOnLostTouch);
}

void ev4sio_Sc::Scene::addStatic(StaticCore& ro, NpShape*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds)
{
	PX_ASSERT(ro.getActorCoreType() == PxActorType::eRIGID_STATIC);

	// sim objects do all the necessary work of adding themselves to broad phase,
	// activation, registering with the interaction system, etc

	StaticSim* sim = mStaticSimPool->construct(*this, ro);
	
	mNbRigidStatics++;
	addShapes(shapes, nbShapes, shapePtrOffset, *sim, uninflatedBounds);
}

void ev4sio_Sc::Scene::removeStatic(StaticCore& ro, PxInlineArray<const ev4sio_Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	PX_ASSERT(ro.getActorCoreType() == PxActorType::eRIGID_STATIC);

	StaticSim* sim = ro.getSim();
	if(sim)
	{
		if(mBatchRemoveState)
		{
			removeShapes(*sim, mBatchRemoveState->bufferedShapes ,removedShapes, wakeOnLostTouch);
		}
		else
		{
			PxInlineArray<ev4sio_Sc::ShapeSim*, 64>  shapesBuffer;
			removeShapes(*sim, shapesBuffer ,removedShapes, wakeOnLostTouch);
		}		
		mStaticSimPool->destroy(static_cast<ev4sio_Sc::StaticSim*>(ro.getSim()));
		mNbRigidStatics--;
	}
}

void ev4sio_Sc::Scene::addBody(BodyCore& body, NpShape*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* outBounds, bool compound)
{
	// sim objects do all the necessary work of adding themselves to broad phase,
	// activation, registering with the interaction system, etc

	BodySim* sim = mBodySimPool->construct(*this, body, compound);

	const bool isArticulationLink = sim->isArticulationLink();

	if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD && sim->isActive())
	{
		if (isArticulationLink)
		{
			if (sim->getNodeIndex().isValid())
				mSpeculativeCDDArticulationBitMap.growAndSet(sim->getNodeIndex().index());
		}
		else
			mSpeculativeCCDRigidBodyBitMap.growAndSet(sim->getNodeIndex().index());
	}
	//if rigid body is articulation link, the node index will be invalid. We should add the link to the scene after we add the
	//articulation for gpu
	if(sim->getNodeIndex().isValid())
		mSimulationController->addDynamic(&sim->getLowLevelBody(), sim->getNodeIndex());
	if(sim->getSimStateData(true) && sim->getSimStateData(true)->isKine())
		mNbRigidKinematic++;
	else
		mNbRigidDynamics++;
	addShapes(shapes, nbShapes, shapePtrOffset, *sim, outBounds);

	mDynamicsContext->setStateDirty(true);
}

void ev4sio_Sc::Scene::removeBody(BodyCore& body, PxInlineArray<const ev4sio_Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	BodySim *sim = body.getSim();	
	if(sim)
	{
		if(mBatchRemoveState)
		{
			removeShapes(*sim, mBatchRemoveState->bufferedShapes, removedShapes, wakeOnLostTouch);
		}
		else
		{
			PxInlineArray<ev4sio_Sc::ShapeSim*, 64>  shapesBuffer;
			removeShapes(*sim,shapesBuffer, removedShapes, wakeOnLostTouch);
		}

		if(!sim->isArticulationLink())
		{
			//clear bit map
			if(sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
				sim->getScene().resetSpeculativeCCDRigidBody(sim->getNodeIndex().index());				
		}
		else
		{
			// PT: TODO: missing call to resetSpeculativeCCDArticulationLink ? 

			sim->getArticulation()->removeBody(*sim);
		}
		if(sim->getSimStateData(true) && sim->getSimStateData(true)->isKine())
		{
			body.onRemoveKinematicFromScene();

			mNbRigidKinematic--;
		}
		else
			mNbRigidDynamics--;

		mBodySimPool->destroy(sim);

		mDynamicsContext->setStateDirty(true);
	}
}

// PT: TODO: refactor with addShapes
void ev4sio_Sc::Scene::addShape_(RigidSim& owner, ShapeCore& shapeCore)
{
	ShapeSim* sim = mShapeSimPool->construct(owner, shapeCore);
	mNbGeometries[shapeCore.getGeometryType()]++;

	//register shape
	mSimulationController->addPxgShape(sim, sim->getPxsShapeCore(), sim->getActorNodeIndex(), sim->getElementID());

	registerShapeInNphase(&owner.getRigidCore(), shapeCore, sim->getElementID());
}

// PT: TODO: refactor with removeShapes
void ev4sio_Sc::Scene::removeShape_(ShapeSim& shape, bool wakeOnLostTouch)
{
	//BodySim* body = shape.getBodySim();
	//if(body)
	//	body->postShapeDetach();
	
	unregisterShapeFromNphase(shape.getCore(), shape.getElementID());

	mSimulationController->removePxgShape(shape.getElementID());

	mNbGeometries[shape.getCore().getGeometryType()]--;
	shape.removeFromBroadPhase(wakeOnLostTouch);
	mShapeSimPool->destroy(&shape);
}

void ev4sio_Sc::Scene::registerShapeInNphase(ev4sio_Sc::RigidCore* rigidCore, const ShapeCore& shape, const PxU32 transformCacheID)
{
	RigidSim* sim = rigidCore->getSim();
	if(sim)
		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), shape.getCore(), transformCacheID, sim->getPxActor());
}

void ev4sio_Sc::Scene::unregisterShapeFromNphase(const ShapeCore& shape, const PxU32 transformCacheID)
{
	mLLContext->getNphaseImplementationContext()->unregisterShape(shape.getCore(), transformCacheID);
}

void ev4sio_Sc::Scene::notifyNphaseOnUpdateShapeMaterial(const ShapeCore& shapeCore)
{
	mLLContext->getNphaseImplementationContext()->updateShapeMaterial(shapeCore.getCore());
}

void ev4sio_Sc::Scene::startBatchInsertion(BatchInsertionState&state)
{
	state.shapeSim = mShapeSimPool->allocateAndPrefetch();
	state.staticSim = mStaticSimPool->allocateAndPrefetch();
	state.bodySim = mBodySimPool->allocateAndPrefetch();														   
}

void ev4sio_Sc::Scene::addShapes(NpShape*const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& rigidSim, ShapeSim*& prefetchedShapeSim, PxBounds3* outBounds)
{
	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(i+1<nbShapes)
			PxPrefetch(shapes[i+1], PxU32(ptrOffset+sizeof(ev4sio_Sc::ShapeCore)));
		ShapeSim* nextShapeSim = mShapeSimPool->allocateAndPrefetch();
		ShapeCore& sc = *PxPointerOffset<ShapeCore*>(shapes[i], ptrdiff_t(ptrOffset));
		PX_PLACEMENT_NEW(prefetchedShapeSim, ShapeSim(rigidSim, sc));
		const PxU32 elementID = prefetchedShapeSim->getElementID();

		outBounds[i] = mBoundsArray->getBounds(elementID);

		// PT: TODO: revisit getActorNodeIndex() vs rigidSim.getNodeIndex()
		mSimulationController->addPxgShape(prefetchedShapeSim, prefetchedShapeSim->getPxsShapeCore(), prefetchedShapeSim->getActorNodeIndex(), elementID);
		mLLContext->getNphaseImplementationContext()->registerShape(rigidSim.getNodeIndex(), sc.getCore(), elementID, rigidSim.getPxActor());

		prefetchedShapeSim = nextShapeSim;
		mNbGeometries[sc.getGeometryType()]++;
	}	
}

void ev4sio_Sc::Scene::addStatic(PxActor* actor, BatchInsertionState& s, PxBounds3* outBounds)
{
	// static core has been prefetched by caller
	ev4sio_Sc::StaticSim* sim = s.staticSim;		// static core has been prefetched by the caller

	const ev4sio_Cm::PtrTable* shapeTable = PxPointerOffset<const ev4sio_Cm::PtrTable*>(actor, s.staticShapeTableOffset);
	void*const* shapes = shapeTable->getPtrs();

	mStaticSimPool->construct(sim, *this, *PxPointerOffset<ev4sio_Sc::StaticCore*>(actor, s.staticActorOffset));
	s.staticSim = mStaticSimPool->allocateAndPrefetch();

	addShapes(reinterpret_cast<NpShape*const*>(shapes), shapeTable->getCount(), size_t(s.shapeOffset), *sim, s.shapeSim, outBounds);
	mNbRigidStatics++;
}

void ev4sio_Sc::Scene::addBody(PxActor* actor, BatchInsertionState& s, PxBounds3* outBounds, bool compound)
{
	ev4sio_Sc::BodySim* sim = s.bodySim;		// body core has been prefetched by the caller

	const ev4sio_Cm::PtrTable* shapeTable = PxPointerOffset<const ev4sio_Cm::PtrTable*>(actor, s.dynamicShapeTableOffset);
	void*const* shapes = shapeTable->getPtrs();

	ev4sio_Sc::BodyCore* bodyCore = PxPointerOffset<ev4sio_Sc::BodyCore*>(actor, s.dynamicActorOffset);
	mBodySimPool->construct(sim, *this, *bodyCore, compound);	
	s.bodySim = mBodySimPool->allocateAndPrefetch();

	if(sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
	{
		if(sim->isArticulationLink())
			mSpeculativeCDDArticulationBitMap.growAndSet(sim->getNodeIndex().index());
		else
			mSpeculativeCCDRigidBodyBitMap.growAndSet(sim->getNodeIndex().index());
	}

	if(sim->getNodeIndex().isValid())
		mSimulationController->addDynamic(&sim->getLowLevelBody(), sim->getNodeIndex());

	addShapes(reinterpret_cast<NpShape*const*>(shapes), shapeTable->getCount(), size_t(s.shapeOffset), *sim, s.shapeSim, outBounds);

	const SimStateData* simStateData = bodyCore->getSim()->getSimStateData(true);
	if(simStateData && simStateData->isKine())
		mNbRigidKinematic++;
	else
		mNbRigidDynamics++;

	mDynamicsContext->setStateDirty(true);
}

void ev4sio_Sc::Scene::finishBatchInsertion(BatchInsertionState& state)
{
	// a little bit lazy - we could deal with the last one in the batch specially to avoid overallocating by one.
	
	mStaticSimPool->releasePreallocated(static_cast<ev4sio_Sc::StaticSim*>(state.staticSim));	
	mBodySimPool->releasePreallocated(static_cast<ev4sio_Sc::BodySim*>(state.bodySim));
	mShapeSimPool->releasePreallocated(state.shapeSim);
}

// PT: TODO: why is this in Sc?
void ev4sio_Sc::Scene::initContactsIterator(ContactIterator& contactIterator, PxsContactManagerOutputIterator& outputs)
{
	outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	ElementSimInteraction** first = mInteractions[ev4sio_Sc::InteractionType::eOVERLAP].begin();
	contactIterator = ContactIterator(first, first + mActiveInteractionCount[ev4sio_Sc::InteractionType::eOVERLAP], outputs);
}

void ev4sio_Sc::Scene::setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)
{
	struct {
		void operator()(PxU32& bits, PxDominanceGroup shift, PxReal weight)
		{
			if(weight != PxReal(0))
				bits |=  (PxU32(1) << shift);
			else 
				bits &= ~(PxU32(1) << shift);
		}
	} bitsetter;

	bitsetter(mDominanceBitMatrix[group1], group2, dominance.dominance0);
	bitsetter(mDominanceBitMatrix[group2], group1, dominance.dominance1);

	mInternalFlags |= SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE;		//force an update on all interactions on matrix change -- very expensive but we have no choice!!
}

PxDominanceGroupPair ev4sio_Sc::Scene::getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const
{
	const PxU8 dom0 = PxU8((mDominanceBitMatrix[group1]>>group2) & 0x1 ? 1u : 0u);
	const PxU8 dom1 = PxU8((mDominanceBitMatrix[group2]>>group1) & 0x1 ? 1u : 0u);
	return PxDominanceGroupPair(dom0, dom1);
}

PxU32 ev4sio_Sc::Scene::getDefaultContactReportStreamBufferSize() const
{
	return mNPhaseCore->getDefaultContactReportStreamBufferSize();
}

void ev4sio_Sc::Scene::buildActiveActors()
{
	{
		PxU32 numActiveBodies = 0;
		BodyCore*const* PX_RESTRICT activeBodies;
		if (!(getFlags() & PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS))
		{
			numActiveBodies = getNumActiveBodies();
			activeBodies = getActiveBodiesArray();
		}
		else
		{
			numActiveBodies = getActiveDynamicBodiesCount();
			activeBodies = getActiveDynamicBodies();
		}

		mActiveActors.clear();

		for (PxU32 i = 0; i < numActiveBodies; i++)
		{
			if (!activeBodies[i]->isFrozen())
			{
				PxRigidActor* ra = static_cast<PxRigidActor*>(activeBodies[i]->getPxActor());
				PX_ASSERT(ra);
				mActiveActors.pushBack(ra);
			}
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	gpu_buildActiveActors();
#endif
}

// PT: TODO: unify buildActiveActors & buildActiveAndFrozenActors
void ev4sio_Sc::Scene::buildActiveAndFrozenActors()
{
	{
		PxU32 numActiveBodies = 0;
		BodyCore*const* PX_RESTRICT activeBodies;
		if (!(getFlags() & PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS))
		{
			numActiveBodies = getNumActiveBodies();
			activeBodies = getActiveBodiesArray();
		}
		else
		{
			numActiveBodies = getActiveDynamicBodiesCount();
			activeBodies = getActiveDynamicBodies();
		}

		mActiveActors.clear();
		mFrozenActors.clear();

		for (PxU32 i = 0; i < numActiveBodies; i++)
		{
			PxRigidActor* ra = static_cast<PxRigidActor*>(activeBodies[i]->getPxActor());
			PX_ASSERT(ra);

			if (!activeBodies[i]->isFrozen())
				mActiveActors.pushBack(ra);
			else
				mFrozenActors.pushBack(ra);
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	gpu_buildActiveAndFrozenActors();
#endif
}

PxActor** ev4sio_Sc::Scene::getActiveActors(PxU32& nbActorsOut)
{
	nbActorsOut = mActiveActors.size();
	
	if(!nbActorsOut)
		return NULL;

	return mActiveActors.begin();
}

void ev4sio_Sc::Scene::setActiveActors(PxActor** actors, PxU32 nbActors)
{
	mActiveActors.forceSize_Unsafe(0);
	mActiveActors.resize(nbActors);
	PxMemCopy(mActiveActors.begin(), actors, sizeof(PxActor*) * nbActors);
}

PxActor** ev4sio_Sc::Scene::getFrozenActors(PxU32& nbActorsOut)
{
	nbActorsOut = mFrozenActors.size();

	if(!nbActorsOut)
		return NULL;

	return mFrozenActors.begin();
}

void ev4sio_Sc::Scene::reserveTriggerReportBufferSpace(const PxU32 pairCount, PxTriggerPair*& triggerPairBuffer, TriggerPairExtraData*& triggerPairExtraBuffer)
{
	const PxU32 oldSize = mTriggerBufferAPI.size();
	const PxU32 newSize = oldSize + pairCount;
	const PxU32 newCapacity = PxU32(newSize * 1.5f);
	mTriggerBufferAPI.reserve(newCapacity);
	mTriggerBufferAPI.forceSize_Unsafe(newSize);
	triggerPairBuffer = mTriggerBufferAPI.begin() + oldSize;

	PX_ASSERT(oldSize == mTriggerBufferExtraData->size());
	mTriggerBufferExtraData->reserve(newCapacity);
	mTriggerBufferExtraData->forceSize_Unsafe(newSize);
	triggerPairExtraBuffer = mTriggerBufferExtraData->begin() + oldSize;
}

template<const bool sleepOrWoke, class T>
static void clearBodies(PxCoalescedHashSet<T*>& bodies)
{
	T* const* sleepingOrWokenBodies = bodies.getEntries();
	const PxU32 nb = bodies.size();
	for(PxU32 i=0; i<nb; i++)
	{
		ActorSim* body = sleepingOrWokenBodies[i]->getSim();

		if(sleepOrWoke)
		{
			PX_ASSERT(!body->readInternalFlag(ActorSim::BF_WAKEUP_NOTIFY));
			body->clearInternalFlag(ActorSim::BF_SLEEP_NOTIFY);
		}
		else
		{
			PX_ASSERT(!body->readInternalFlag(ActorSim::BF_SLEEP_NOTIFY));
			body->clearInternalFlag(ActorSim::BF_WAKEUP_NOTIFY);
		}

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST);
		body->clearInternalFlag(ActorSim::BF_IS_IN_WAKEUP_LIST);
	}
	bodies.clear();
}

void ev4sio_Sc::Scene::clearSleepWakeBodies()
{
	// Clear sleep/woken marker flags
	clearBodies<true>(mSleepBodies);
	clearBodies<false>(mWokeBodies);

	mSleepBodies.clear();
	mWokeBodies.clear();
	mWokeBodyListValid = true;
	mSleepBodyListValid = true;

#if PX_SUPPORT_GPU_PHYSX
	gpu_clearSleepWakeBodies();
#endif
}

void ev4sio_Sc::Scene::onBodySleep(BodySim* body)
{
	if (!mSimulationEventCallback && !mOnSleepingStateChanged)
		return;

	if (body->readInternalFlag(ActorSim::BF_WAKEUP_NOTIFY))
	{
		PX_ASSERT(!body->readInternalFlag(ActorSim::BF_SLEEP_NOTIFY));

		// Body is in the list of woken bodies, hence, mark this list as dirty such that it gets cleaned up before
		// being sent to the user
		body->clearInternalFlag(ActorSim::BF_WAKEUP_NOTIFY);
		mWokeBodyListValid = false;
	}

	body->raiseInternalFlag(ActorSim::BF_SLEEP_NOTIFY);

	// Avoid multiple insertion (the user can do multiple transitions between asleep and awake)
	if (!body->readInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST))
	{
		PX_ASSERT(!mSleepBodies.contains(&body->getBodyCore()));
		mSleepBodies.insert(&body->getBodyCore());
		body->raiseInternalFlag(ActorSim::BF_IS_IN_SLEEP_LIST);
	}
}

void ev4sio_Sc::Scene::onBodyWakeUp(BodySim* body)
{
	if(!mSimulationEventCallback && !mOnSleepingStateChanged)
		return;

	if (body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY))
	{
		PX_ASSERT(!body->readInternalFlag(BodySim::BF_WAKEUP_NOTIFY));

		// Body is in the list of sleeping bodies, hence, mark this list as dirty such it gets cleaned up before
		// being sent to the user
		body->clearInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		mSleepBodyListValid = false;
	}

	body->raiseInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

	// Avoid multiple insertion (the user can do multiple transitions between asleep and awake)
	if (!body->readInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST))
	{
		PX_ASSERT(!mWokeBodies.contains(&body->getBodyCore()));
		mWokeBodies.insert(&body->getBodyCore());
		body->raiseInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}
}

PX_INLINE void ev4sio_Sc::Scene::cleanUpSleepBodies()
{
	BodyCore* const* bodyArray = mSleepBodies.getEntries();
	PxU32 bodyCount = mSleepBodies.size();

	ev4sio_IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	while (bodyCount--)
	{
		ActorSim* actor = bodyArray[bodyCount]->getSim();
		BodySim* body = static_cast<BodySim*>(actor);

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_WAKEUP_NOTIFY)))
		{
			body->clearInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_IS_IN_WAKEUP_LIST));
			mSleepBodies.erase(bodyArray[bodyCount]);
		}
		else if (islandSim.getNode(body->getNodeIndex()).isActive())
		{
			//This body is still active in the island simulation, so the request to deactivate the actor by the application must have failed. Recover by undoing this
			mSleepBodies.erase(bodyArray[bodyCount]);
			actor->internalWakeUp();

		}
	}

	mSleepBodyListValid = true;
}

PX_INLINE void ev4sio_Sc::Scene::cleanUpWokenBodies()
{
	cleanUpSleepOrWokenBodies(mWokeBodies, BodySim::BF_SLEEP_NOTIFY, mWokeBodyListValid);
}

PX_INLINE void ev4sio_Sc::Scene::cleanUpSleepOrWokenBodies(PxCoalescedHashSet<BodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker)
{
	// With our current logic it can happen that a body is added to the sleep as well as the woken body list in the
	// same frame.
	//
	// Examples:
	// - Kinematic is created (added to woken list) but has not target (-> deactivation -> added to sleep list)
	// - Dynamic is created (added to woken list) but is forced to sleep by user (-> deactivation -> added to sleep list)
	//
	// This code traverses the sleep/woken body list and removes bodies which have been initially added to the given
	// list but do not belong to it anymore.

	BodyCore* const* bodyArray = bodyList.getEntries();
	PxU32 bodyCount = bodyList.size();
	while (bodyCount--)
	{
		BodySim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(removeFlag)))
			bodyList.erase(bodyArray[bodyCount]);
	}

	validMarker = true;
}

FeatherstoneArticulation* ev4sio_Sc::Scene::createLLArticulation(ev4sio_Sc::ArticulationSim* sim)
{
	return mLLArticulationRCPool->construct(sim);
}

void ev4sio_Sc::Scene::destroyLLArticulation(FeatherstoneArticulation& articulation)
{
	mLLArticulationRCPool->destroy(static_cast<ev4sio_Dy::FeatherstoneArticulation*>(&articulation));
}

PxU32 ev4sio_Sc::Scene::createAggregate(void* userData, PxU32 maxNumShapes, PxAggregateFilterHint filterHint, PxU32 envID)
{
	const ev4sio_Bp::BoundsIndex index = getElementIDPool().createID();
	mBoundsArray->initEntry(index);
	mLLContext->getNphaseImplementationContext()->registerAggregate(index);
#if BP_USE_AGGREGATE_GROUP_TAIL
	return mAABBManager->createAggregate(index, ev4sio_Bp::FilterGroup::eINVALID, userData, maxNumShapes, filterHint, envID);
#else
	// PT: TODO: ideally a static compound would have a static group
	const PxU32 rigidId	= getRigidIDTracker().createID();
	const ev4sio_Bp::FilterGroup::Enum bpGroup = ev4sio_Bp::FilterGroup::Enum(rigidId + ev4sio_Bp::FilterGroup::eDYNAMICS_BASE);
	return mAABBManager->createAggregate(index, bpGroup, userData, maxNumShapes, filterHint, envID);
#endif
}

void ev4sio_Sc::Scene::deleteAggregate(PxU32 id)
{
	ev4sio_Bp::BoundsIndex index;
	ev4sio_Bp::FilterGroup::Enum bpGroup;
#if BP_USE_AGGREGATE_GROUP_TAIL
	if(mAABBManager->destroyAggregate(index, bpGroup, id))
	{
		getElementIDPool().releaseID(index);
	}
#else
	if(mAABBManager->destroyAggregate(index, bpGroup, id))
	{
		getElementIDPool().releaseID(index);

		// PT: this is clumsy....
		const PxU32 rigidId	= PxU32(bpGroup) - ev4sio_Bp::FilterGroup::eDYNAMICS_BASE;
		getRigidIDTracker().releaseID(rigidId);
	}
#endif
}

void ev4sio_Sc::Scene::shiftOrigin(const PxVec3& shift)
{
	// adjust low level context
	mLLContext->shiftOrigin(shift);

	// adjust bounds array
	mBoundsArray->shiftOrigin(shift);

	// adjust broadphase
	mAABBManager->shiftOrigin(shift);

	// adjust constraints
	ConstraintCore*const * constraints = mConstraints.getEntries();
	for(PxU32 i=0, size = mConstraints.size(); i < size; i++)
		constraints[i]->getPxConnector()->onOriginShift(shift);
}

///////////////////////////////////////////////////////////////////////////////

// PT: onActivate() functions should be called when an interaction is activated or created, and return true if activation
// should proceed else return false (for example: joint interaction between two kinematics should not get activated)
bool ev4sio_Sc::activateInteraction(ev4sio_Sc::Interaction* interaction)
{
	switch(interaction->getType())
	{
		case InteractionType::eOVERLAP:
			return static_cast<ev4sio_Sc::ShapeInteraction*>(interaction)->onActivate(NULL);

		case InteractionType::eTRIGGER:
			return static_cast<ev4sio_Sc::TriggerInteraction*>(interaction)->onActivate();

		case InteractionType::eMARKER:
			// PT: ElementInteractionMarker::onActivate() always returns false (always inactive).
			return false;

		case InteractionType::eCONSTRAINTSHADER:
			return static_cast<ev4sio_Sc::ConstraintInteraction*>(interaction)->onActivate();

		case InteractionType::eARTICULATION:
			return static_cast<ev4sio_Sc::ArticulationJointSim*>(interaction)->onActivate();

		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
		PX_ASSERT(0);
		break;
	}
	return false;
}

// PT: onDeactivate() functions should be called when an interaction is deactivated, and return true if deactivation should proceed
// else return false (for example: joint interaction between two kinematics can ignore deactivation because it always is deactivated)
/*static*/ bool deactivateInteraction(ev4sio_Sc::Interaction* interaction, const ev4sio_Sc::InteractionType::Enum type)
{
	switch(type)
	{
		case InteractionType::eOVERLAP:
			return static_cast<ev4sio_Sc::ShapeInteraction*>(interaction)->onDeactivate();

		case InteractionType::eTRIGGER:
			return static_cast<ev4sio_Sc::TriggerInteraction*>(interaction)->onDeactivate();

		case InteractionType::eMARKER:
			// PT: ElementInteractionMarker::onDeactivate() always returns true.
			return true;

		case InteractionType::eCONSTRAINTSHADER:
			return static_cast<ev4sio_Sc::ConstraintInteraction*>(interaction)->onDeactivate();

		case InteractionType::eARTICULATION:
			return static_cast<ev4sio_Sc::ArticulationJointSim*>(interaction)->onDeactivate();

		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
		PX_ASSERT(0);
		break;
	}
	return false;
}

void ev4sio_Sc::activateInteractions(ev4sio_Sc::ActorSim& actorSim)
{
	const PxU32 nbInteractions = actorSim.getActorInteractionCount();
	if(!nbInteractions)
		return;

	Interaction** interactions = actorSim.getActorInteractions();
	Scene& scene = actorSim.getScene();

	for(PxU32 i=0; i<nbInteractions; ++i)
	{
		PxPrefetchLine(interactions[PxMin(i+1,nbInteractions-1)]);
		Interaction* interaction = interactions[i];

		if(!interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			const InteractionType::Enum type = interaction->getType();
			const bool isNotIGControlled = type != InteractionType::eOVERLAP && type != InteractionType::eMARKER;

			if(isNotIGControlled)
			{
				const bool proceed = activateInteraction(interaction);
				if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
					scene.notifyInteractionActivated(interaction);	// PT: we can reach this line for trigger interactions
			}
		}
	}
}

void ev4sio_Sc::deactivateInteractions(ev4sio_Sc::ActorSim& actorSim)
{
	const PxU32 nbInteractions = actorSim.getActorInteractionCount();
	if(!nbInteractions)
		return;

	Interaction** interactions = actorSim.getActorInteractions();
	Scene& scene = actorSim.getScene();

	for(PxU32 i=0; i<nbInteractions; ++i)
	{
		PxPrefetchLine(interactions[PxMin(i+1,nbInteractions-1)]);
		Interaction* interaction = interactions[i];

		if(interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
		{
			const InteractionType::Enum type = interaction->getType();
			const bool isNotIGControlled = type != InteractionType::eOVERLAP && type != InteractionType::eMARKER;
			if(isNotIGControlled)
			{
				const bool proceed = deactivateInteraction(interaction, type);
				if(proceed && (type < InteractionType::eTRACKED_IN_SCENE_COUNT))
					scene.notifyInteractionDeactivated(interaction);	// PT: we can reach this line for trigger interactions
			}
		}
	}
}

ev4sio_Sc::ConstraintCore*	ev4sio_Sc::Scene::findConstraintCore(const ev4sio_Sc::ActorSim* sim0, const ev4sio_Sc::ActorSim* sim1)
{
	const PxNodeIndex ind0 = sim0->getNodeIndex();
	const PxNodeIndex ind1 = sim1->getNodeIndex();

	if(ind1 < ind0)
		PxSwap(sim0, sim1);

	const PxHashMap<PxPair<const ev4sio_Sc::ActorSim*, const ev4sio_Sc::ActorSim*>, ev4sio_Sc::ConstraintCore*>::Entry* entry = mConstraintMap.find(PxPair<const ev4sio_Sc::ActorSim*, const ev4sio_Sc::ActorSim*>(sim0, sim1));
	return entry ? entry->second : NULL;
}

void ev4sio_Sc::Scene::updateBodySim(ev4sio_Sc::BodySim& bodySim)
{
	ev4sio_Dy::FeatherstoneArticulation* arti = NULL;
	ev4sio_Sc::ArticulationSim* artiSim = bodySim.getArticulation();
	if (artiSim)
		arti = artiSim->getLowLevelArticulation();
	mSimulationController->updateDynamic(arti, bodySim.getNodeIndex());
}

// PT: start moving PX_SUPPORT_GPU_PHYSX bits to the end of the file. Ideally/eventually they would move to a separate class or file,
// to clearly decouple the CPU and GPU parts of the scene/pipeline.
#if PX_SUPPORT_GPU_PHYSX
void ev4sio_Sc::Scene::gpu_releasePools()
{
	PX_DELETE(mLLDeformableSurfacePool);
	PX_DELETE(mLLDeformableVolumePool);
	PX_DELETE(mLLParticleSystemPool);
}

void ev4sio_Sc::Scene::gpu_release()
{
	PX_DELETE(mHeapMemoryAllocationManager);
}

template<class T>
static void addToActiveArray(PxArray<T*>& activeArray, ActorSim& actorSim, ActorCore* core)
{
	const PxU32 activeListIndex = activeArray.size();
	actorSim.setActiveListIndex(activeListIndex);
	activeArray.pushBack(static_cast<T*>(core));
}

void ev4sio_Sc::Scene::gpu_addToActiveList(ActorSim& actorSim, ActorCore* appendedActorCore)
{
	if (actorSim.isDeformableSurface())
		addToActiveArray(mActiveDeformableSurfaces, actorSim, appendedActorCore);
	else if (actorSim.isDeformableVolume())
		addToActiveArray(mActiveDeformableVolumes, actorSim, appendedActorCore);
	else if (actorSim.isParticleSystem())
		addToActiveArray(mActiveParticleSystems, actorSim, appendedActorCore);
}

template<class T>
static void removeFromActiveArray(PxArray<T*>& activeArray, PxU32 removedActiveIndex)
{
	const PxU32 newSize = activeArray.size() - 1;

	if(removedActiveIndex != newSize)
	{
		T* lastBody = activeArray[newSize];
		activeArray[removedActiveIndex] = lastBody;
		lastBody->getSim()->setActiveListIndex(removedActiveIndex);
	}
	activeArray.forceSize_Unsafe(newSize);
}

void ev4sio_Sc::Scene::gpu_removeFromActiveList(ActorSim& actorSim, PxU32 removedActiveIndex)
{
	if(actorSim.isDeformableSurface())
		removeFromActiveArray(mActiveDeformableSurfaces, removedActiveIndex);
	else if(actorSim.isDeformableVolume())
		removeFromActiveArray(mActiveDeformableVolumes, removedActiveIndex);
	else if(actorSim.isParticleSystem())
		removeFromActiveArray(mActiveParticleSystems, removedActiveIndex);
}

void ev4sio_Sc::Scene::gpu_clearSleepWakeBodies()
{
	clearBodies<true>(mSleepDeformableVolumes);
	clearBodies<false>(mWokeDeformableVolumes);

	mWokeDeformableVolumeListValid = true;
	mSleepDeformableVolumeListValid = true;
}

void ev4sio_Sc::Scene::gpu_buildActiveActors()
{
	{
		PxU32 numActiveDeformableVolumes = getNumActiveDeformableVolumes();
		DeformableVolumeCore*const* PX_RESTRICT activeDeformableVolumes = getActiveDeformableVolumesArray();

		mActiveDeformableVolumeActors.clear();

		for (PxU32 i = 0; i < numActiveDeformableVolumes; i++)
		{
			PxActor* ra = activeDeformableVolumes[i]->getPxActor();
			mActiveDeformableVolumeActors.pushBack(ra);
		}
	}
}

void ev4sio_Sc::Scene::gpu_buildActiveAndFrozenActors()
{
	{
		PxU32 numActiveDeformableVolumes = getNumActiveDeformableVolumes();
		DeformableVolumeCore*const* PX_RESTRICT activeDeformableVolumes = getActiveDeformableVolumesArray();
		
		mActiveDeformableVolumeActors.clear();

		for (PxU32 i = 0; i < numActiveDeformableVolumes; i++)
		{
			PxActor* ra = activeDeformableVolumes[i]->getPxActor();
			mActiveDeformableVolumeActors.pushBack(ra);
		}
	}
}

void ev4sio_Sc::Scene::gpu_setSimulationEventCallback(PxSimulationEventCallback* /*callback*/)
{
	DeformableVolumeCore* const* sleepingDeformableVolumes = mSleepDeformableVolumes.getEntries();
	for (PxU32 i = 0; i < mSleepDeformableVolumes.size(); i++)
	{
		sleepingDeformableVolumes[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
	}

	//DeformableSurfaceCore* const* sleepingDeformableSurfaces = mSleepDeformableSurfaces.getEntries();
	//for (PxU32 i = 0; i < mSleepDeformableSurfaces.size(); i++)
	//{
	//	sleepingDeformableSurfaces[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
	//}
}

PxU32 ev4sio_Sc::Scene::gpu_cleanUpSleepAndWokenBodies()
{
	if (!mSleepDeformableVolumeListValid)
		cleanUpSleepDeformableVolumes();

	if (!mWokeBodyListValid)
		cleanUpWokenDeformableVolumes();

	const PxU32 nbVolumeSleep = mSleepDeformableVolumes.size();
	const PxU32 nbVolumeWoken = mWokeDeformableVolumes.size();
	return PxMax(nbVolumeWoken, nbVolumeSleep);
}

static void gpu_fireSleepingCallback(	PxActor** actors, const PxCoalescedHashSet<ev4sio_Sc::DeformableVolumeCore*>& bodies,
										PxSimulationEventCallback* simulationEventCallback, PxU64 contextID,
										Scene::SleepingStateChangedCallback onSleepingStateChanged, bool sleeping)
{
	PX_UNUSED(contextID);
	const PxU32 nbBodies = bodies.size();
	if (nbBodies)
	{
		PxU32 destSlot = 0;
		ev4sio_Sc::DeformableVolumeCore* const* sleepingDeformableVolumes = bodies.getEntries();
		for (PxU32 i = 0; i<nbBodies; i++)
		{
			ev4sio_Sc::DeformableVolumeCore* body = sleepingDeformableVolumes[i];
			if (body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
				actors[destSlot++] = body->getPxActor();
			if (onSleepingStateChanged)
				onSleepingStateChanged(*static_cast<PxRigidDynamic*>(body->getPxActor()), sleeping);
		}

		if (destSlot && simulationEventCallback)
		{
			if(sleeping)
			{
				PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onSleep", contextID);
				simulationEventCallback->onSleep(actors, destSlot);
			}
			else
			{
				PX_PROFILE_ZONE("USERCODE - PxSimulationEventCallback::onWake", contextID);
				simulationEventCallback->onWake(actors, destSlot);
			}
		}
	}
}

void ev4sio_Sc::Scene::gpu_fireOnSleepCallback(PxActor** actors)
{
	//ML: need to create and API for the onSleep for deformable volume
	gpu_fireSleepingCallback(actors, mSleepDeformableVolumes, mSimulationEventCallback, mContextId, mOnSleepingStateChanged, true);
}

void ev4sio_Sc::Scene::gpu_fireOnWakeCallback(PxActor** actors)
{
	//ML: need to create an API for woken deformable volume
	gpu_fireSleepingCallback(actors, mWokeDeformableVolumes, mSimulationEventCallback, mContextId, mOnSleepingStateChanged, false);
}

void ev4sio_Sc::Scene::gpu_updateBounds()
{
	bool gpuStateChanged = false;

	PxBitMapPinned& changedMap = mAABBManager->getChangedAABBMgActorHandleMap();

	//update deformable volumes world bound
	ev4sio_Sc::DeformableVolumeCore* const* deformableVolumes = mDeformableVolumes.getEntries();
	PxU32 size = mDeformableVolumes.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < size; ++i)
			changedMap.growAndSet(deformableVolumes[i]->getSim()->getShapeSim().getElementID());

		if(size)
			gpuStateChanged = true;
	}
	else
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			DeformableVolumeSim* volumeSim = deformableVolumes[i]->getSim();
			ShapeSimBase& shapeSim = volumeSim->getShapeSim();

			PxBounds3 worldBounds = volumeSim->getWorldBounds();
			worldBounds.fattenSafe(shapeSim.getContactOffset()); // fatten for fast moving colliders
			mBoundsArray->setBounds(worldBounds, shapeSim.getElementID());
			changedMap.growAndSet(shapeSim.getElementID());
		}
	}

	// update FEM-cloth world bound
	ev4sio_Sc::DeformableSurfaceCore* const* deformableSurfaces = mDeformableSurfaces.getEntries();
	size = mDeformableSurfaces.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < size; ++i)
			changedMap.growAndSet(deformableSurfaces[i]->getSim()->getShapeSim().getElementID());

		if(size)
			gpuStateChanged = true;
	}
	else
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			ev4sio_Sc::DeformableSurfaceSim* surfaceSim = deformableSurfaces[i]->getSim();
			ShapeSimBase& shapeSim = surfaceSim->getShapeSim();

			PxBounds3 worldBounds = surfaceSim->getWorldBounds();
			worldBounds.fattenSafe(shapeSim.getContactOffset()); // fatten for fast moving colliders
			mBoundsArray->setBounds(worldBounds, shapeSim.getElementID());
			changedMap.growAndSet(shapeSim.getElementID());
		}
	}

	//upate the actor handle of particle system in AABB manager 
	ev4sio_Sc::ParticleSystemCore* const* particleSystems = mParticleSystems.getEntries();

	size = mParticleSystems.size();
	if (mUseGpuBp)
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			ev4sio_Sc::ShapeSimBase& ps = particleSystems[i]->getSim()->getShapeSim();

			//we are updating the bound in GPU so we just need to set the actor handle in CPU to make sure
			//the GPU BP will process the particles
			if (!(static_cast<ev4sio_Sc::ParticleSystemSim&>(ps.getActor()).getCore().getFlags() & PxParticleFlag::eDISABLE_RIGID_COLLISION))
			{
				changedMap.growAndSet(ps.getElementID());
				gpuStateChanged = true;
			}
		}

		if(gpuStateChanged)
			mAABBManager->setGPUStateChanged();
	}
	else
	{
		for (PxU32 i = 0; i < size; ++i)
		{
			ShapeSimBase& shapeSim = particleSystems[i]->getSim()->getShapeSim();

			const PxVec3 offset(shapeSim.getContactOffset());	// fatten for fast moving colliders
			mBoundsArray->setBounds(PxBounds3(-offset, offset), shapeSim.getElementID());
			changedMap.growAndSet(shapeSim.getElementID());
		}
	}
}

void ev4sio_Sc::Scene::addDeformableSurface(DeformableSurfaceCore& deformableSurface)
{
	DeformableSurfaceSim* sim = PX_NEW(DeformableSurfaceSim)(deformableSurface, *this);

	if (sim && (sim->getLowLevelDeformableSurface() == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mDeformableSurfaces.insert(&deformableSurface);
	mStats->gpuMemSizeDeformableSurfaces += deformableSurface.getGpuMemStat();
}

void ev4sio_Sc::Scene::removeDeformableSurface(DeformableSurfaceCore& deformableSurface)
{
	DeformableSurfaceSim* a = deformableSurface.getSim();
	PX_DELETE(a);
	mDeformableSurfaces.erase(&deformableSurface);
	mStats->gpuMemSizeDeformableSurfaces -= deformableSurface.getGpuMemStat();
}

void ev4sio_Sc::Scene::addDeformableVolume(DeformableVolumeCore& deformableVolume)
{
	DeformableVolumeSim* sim = PX_NEW(DeformableVolumeSim)(deformableVolume, *this);

	if (sim && (sim->getLowLevelDeformableVolume() == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mDeformableVolumes.insert(&deformableVolume);
	mStats->gpuMemSizeDeformableVolumes += deformableVolume.getGpuMemStat();
}

void ev4sio_Sc::Scene::removeDeformableVolume(DeformableVolumeCore& deformableVolume)
{
	DeformableVolumeSim* a = deformableVolume.getSim();
	PX_DELETE(a);
	mDeformableVolumes.erase(&deformableVolume);
	mStats->gpuMemSizeDeformableVolumes -= deformableVolume.getGpuMemStat();
}

void ev4sio_Sc::Scene::addParticleSystem(ParticleSystemCore& particleSystem)
{
	ParticleSystemSim* sim = PX_NEW(ParticleSystemSim)(particleSystem, *this);

	ev4sio_Dy::ParticleSystem* dyParticleSystem = sim->getLowLevelParticleSystem();

	if (sim && (dyParticleSystem == NULL))
	{
		PX_DELETE(sim);
		return;
	}

	mParticleSystems.insert(&particleSystem);
	mStats->gpuMemSizeParticles += particleSystem.getShapeCore().getGpuMemStat();
}

void ev4sio_Sc::Scene::removeParticleSystem(ParticleSystemCore& particleSystem)
{
	ParticleSystemSim* a = particleSystem.getSim();
	PX_DELETE(a);
	mParticleSystems.erase(&particleSystem);
	mStats->gpuMemSizeParticles -= particleSystem.getShapeCore().getGpuMemStat();
}

ev4sio_Dy::DeformableSurface* ev4sio_Sc::Scene::createLLDeformableSurface(ev4sio_Sc::DeformableSurfaceSim* sim)
{
	return mLLDeformableSurfacePool->construct(sim, sim->getCore().getCore());
}

void ev4sio_Sc::Scene::destroyLLDeformableSurface(ev4sio_Dy::DeformableSurface& deformableSurface)
{
	mLLDeformableSurfacePool->destroy(&deformableSurface);
}

ev4sio_Dy::DeformableVolume* ev4sio_Sc::Scene::createLLDeformableVolume(ev4sio_Sc::DeformableVolumeSim* sim)
{
	return mLLDeformableVolumePool->construct(sim, sim->getCore().getCore());
}

void ev4sio_Sc::Scene::destroyLLDeformableVolume(ev4sio_Dy::DeformableVolume& deformableVolume)
{
	mLLDeformableVolumePool->destroy(&deformableVolume);
}

ev4sio_Dy::ParticleSystem*	ev4sio_Sc::Scene::createLLParticleSystem(ev4sio_Sc::ParticleSystemSim* sim)
{
	return mLLParticleSystemPool->construct(sim->getCore().getShapeCore().getLLCore());
}

void ev4sio_Sc::Scene::destroyLLParticleSystem(ev4sio_Dy::ParticleSystem& particleSystem)
{
	return mLLParticleSystemPool->destroy(&particleSystem);
}

PX_INLINE void ev4sio_Sc::Scene::cleanUpSleepDeformableVolumes()
{
	DeformableVolumeCore* const* bodyArray = mSleepDeformableVolumes.getEntries();
	PxU32 bodyCount = mSleepBodies.size();

	ev4sio_IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	while (bodyCount--)
	{
		DeformableVolumeSim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_WAKEUP_NOTIFY)))
		{
			body->clearInternalFlag(static_cast<BodySim::InternalFlags>(ActorSim::BF_IS_IN_WAKEUP_LIST));
			mSleepDeformableVolumes.erase(bodyArray[bodyCount]);
		}
		else if (islandSim.getNode(body->getNodeIndex()).isActive())
		{
			//This body is still active in the island simulation, so the request to deactivate the actor by the application must have failed. Recover by undoing this
			mSleepDeformableVolumes.erase(bodyArray[bodyCount]);
			body->internalWakeUp();
		}
	}

	mSleepBodyListValid = true;
}

PX_INLINE void ev4sio_Sc::Scene::cleanUpWokenDeformableVolumes()
{
	cleanUpSleepOrWokenDeformableVolumes(mWokeDeformableVolumes, BodySim::BF_SLEEP_NOTIFY, mWokeDeformableVolumeListValid);
}

PX_INLINE void ev4sio_Sc::Scene::cleanUpSleepOrWokenDeformableVolumes(PxCoalescedHashSet<DeformableVolumeCore*>& bodyList, PxU32 removeFlag, bool& validMarker)
{
	// With our current logic it can happen that a body is added to the sleep as well as the woken body list in the
	// same frame.
	//
	// Examples:
	// - Kinematic is created (added to woken list) but has not target (-> deactivation -> added to sleep list)
	// - Dynamic is created (added to woken list) but is forced to sleep by user (-> deactivation -> added to sleep list)
	//
	// This code traverses the sleep/woken body list and removes bodies which have been initially added to the given
	// list but do not belong to it anymore.

	DeformableVolumeCore* const* bodyArray = bodyList.getEntries();
	PxU32 bodyCount = bodyList.size();
	while (bodyCount--)
	{
		DeformableVolumeSim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(removeFlag)))
			bodyList.erase(bodyArray[bodyCount]);
	}

	validMarker = true;
}

void ev4sio_Sc::Scene::addDeformableSurfaceSimControl(ev4sio_Sc::DeformableSurfaceCore& core)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addFEMCloth(sim->getLowLevelDeformableSurface(), sim->getNodeIndex());

		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), sim->getPxActor(), true);
	}
}

void ev4sio_Sc::Scene::removeDeformableSurfaceSimControl(ev4sio_Sc::DeformableSurfaceCore& core)
{
	ev4sio_Sc::DeformableSurfaceSim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), true);
		mSimulationController->releaseFEMCloth(sim->getLowLevelDeformableSurface());
	}
}

void ev4sio_Sc::Scene::addDeformableVolumeSimControl(ev4sio_Sc::DeformableVolumeCore& core)
{
	ev4sio_Sc::DeformableVolumeSim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addSoftBody(sim->getLowLevelDeformableVolume(), sim->getNodeIndex());

		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID(), sim->getPxActor());
	}
}

void ev4sio_Sc::Scene::removeDeformableVolumeSimControl(ev4sio_Sc::DeformableVolumeCore& core)
{
	ev4sio_Sc::DeformableVolumeSim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getShapeSim().getCore().getCore(), sim->getShapeSim().getElementID());
		mSimulationController->releaseSoftBody(sim->getLowLevelDeformableVolume());
	}
}

void ev4sio_Sc::Scene::addParticleFilter(ev4sio_Sc::ParticleSystemCore* core, DeformableVolumeSim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	mSimulationController->addParticleFilter(sim.getLowLevelDeformableVolume(), core->getSim()->getLowLevelParticleSystem(),
		particleId, userBufferId, tetId);
}

void ev4sio_Sc::Scene::removeParticleFilter(ev4sio_Sc::ParticleSystemCore* core, DeformableVolumeSim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)
{
	mSimulationController->removeParticleFilter(sim.getLowLevelDeformableVolume(), core->getSim()->getLowLevelParticleSystem(), particleId, userBufferId, tetId);
}

static PX_FORCE_INLINE void addToIslandManager(
	PxHashMap<PxPair<PxU32, PxU32>, ParticleOrSoftBodyRigidInteraction>& particleOrSoftBodyRigidInteractionMap,
	ev4sio_IG::SimpleIslandManager* islandManager, const ActorSim& sim, PxNodeIndex nodeIndex, ev4sio_IG::Edge::EdgeType edgeType)
{
	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = particleOrSoftBodyRigidInteractionMap[pair];

	if (interaction.mCount == 0)
	{
		// PT: TODO: clarify why they do these IM calls exactly
		const ev4sio_IG::EdgeIndex edgeIdx = islandManager->addContactManager(NULL, sim.getNodeIndex(), nodeIndex, NULL, edgeType);
		islandManager->setEdgeConnected(edgeIdx, edgeType);
		interaction.mIndex = edgeIdx;
	}
	interaction.mCount++;
}

static PX_FORCE_INLINE void removeFromIslandManager(
	PxHashMap<PxPair<PxU32, PxU32>, ParticleOrSoftBodyRigidInteraction>& particleOrSoftBodyRigidInteractionMap,
	ev4sio_IG::SimpleIslandManager* islandManager, const ActorSim& sim, PxNodeIndex nodeIndex)
{
	PxPair<PxU32, PxU32> pair(sim.getNodeIndex().index(), nodeIndex.index());
	ParticleOrSoftBodyRigidInteraction& interaction = particleOrSoftBodyRigidInteractionMap[pair];
	interaction.mCount--;
	if (interaction.mCount == 0)
	{
		islandManager->removeConnection(interaction.mIndex);
		particleOrSoftBodyRigidInteractionMap.erase(pair);
	}
}

PxU32 ev4sio_Sc::Scene::addParticleAttachment(ev4sio_Sc::ParticleSystemCore* core, DeformableVolumeSim& sim, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric)
{
	PxNodeIndex nodeIndex = core->getSim()->getNodeIndex();

	PxU32 handle = mSimulationController->addParticleAttachment(sim.getLowLevelDeformableVolume(), core->getSim()->getLowLevelParticleSystem(),
		particleId, userBufferId, tetId, barycentric, sim.isActive());

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex, ev4sio_IG::Edge::eSOFT_BODY_CONTACT);
	return handle;
}

void ev4sio_Sc::Scene::removeParticleAttachment(ev4sio_Sc::ParticleSystemCore* core, DeformableVolumeSim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex = core->getSim()->getNodeIndex();
	
	mSimulationController->removeParticleAttachment(sim.getLowLevelDeformableVolume(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex);
}

void ev4sio_Sc::Scene::addRigidFilter(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 vertId)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addRigidFilter(sim.getLowLevelDeformableVolume(), nodeIndex, vertId);
}

void ev4sio_Sc::Scene::removeRigidFilter(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 vertId)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidFilter(sim.getLowLevelDeformableVolume(), nodeIndex, vertId);
}

PxU32 ev4sio_Sc::Scene::addRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 vertId, const PxVec3& actorSpacePose,
	PxConeLimitedConstraint* constraint, bool doConversion)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addRigidAttachment(sim.getLowLevelDeformableVolume(), sim.getNodeIndex(), body, 
		nodeIndex, vertId, actorSpacePose, constraint, sim.isActive(), doConversion);

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex, ev4sio_IG::Edge::eSOFT_BODY_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::removeRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex;
	
	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidAttachment(sim.getLowLevelDeformableVolume(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex);
}

void ev4sio_Sc::Scene::addTetRigidFilter(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addTetRigidFilter(sim.getLowLevelDeformableVolume(), nodeIndex, tetIdx);
}

void ev4sio_Sc::Scene::removeTetRigidFilter(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}
	mSimulationController->removeTetRigidFilter(sim.getLowLevelDeformableVolume(), nodeIndex, tetIdx);
}

PxU32 ev4sio_Sc::Scene::addTetRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, 
	PxConeLimitedConstraint* constraint, bool doConversion)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addTetRigidAttachment(sim.getLowLevelDeformableVolume(), body, nodeIndex,
		tetIdx, barycentric, actorSpacePose, constraint, sim.isActive(), doConversion);

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex, ev4sio_IG::Edge::eSOFT_BODY_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::addSoftBodyFilter(DeformableVolumeCore& core, PxU32 tetIdx0, DeformableVolumeSim& sim, PxU32 tetIdx1)
{
	ev4sio_Sc::DeformableVolumeSim& bSim = *core.getSim();

	mSimulationController->addSoftBodyFilter(bSim.getLowLevelDeformableVolume(), sim.getLowLevelDeformableVolume(), tetIdx0, tetIdx1);
}

void ev4sio_Sc::Scene::removeSoftBodyFilter(DeformableVolumeCore& core, PxU32 tetIdx0, DeformableVolumeSim& sim, PxU32 tetIdx1)
{
	ev4sio_Sc::DeformableVolumeSim& bSim = *core.getSim();
	mSimulationController->removeSoftBodyFilter(bSim.getLowLevelDeformableVolume(), sim.getLowLevelDeformableVolume(), tetIdx0, tetIdx1);
}

void ev4sio_Sc::Scene::addSoftBodyFilters(DeformableVolumeCore& core, DeformableVolumeSim& sim, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	ev4sio_Sc::DeformableVolumeSim& bSim = *core.getSim();

	mSimulationController->addSoftBodyFilters(bSim.getLowLevelDeformableVolume(), sim.getLowLevelDeformableVolume(), tetIndices0, tetIndices1, tetIndicesSize);
}

void ev4sio_Sc::Scene::removeSoftBodyFilters(DeformableVolumeCore& core, DeformableVolumeSim& sim, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	ev4sio_Sc::DeformableVolumeSim& bSim = *core.getSim();
	mSimulationController->removeSoftBodyFilters(bSim.getLowLevelDeformableVolume(), sim.getLowLevelDeformableVolume(), tetIndices0, tetIndices1, tetIndicesSize);
}

PxU32 ev4sio_Sc::Scene::addSoftBodyAttachment(DeformableVolumeCore& core, PxU32 tetIdx0, const PxVec4& tetBarycentric0, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
	PxConeLimitedConstraint* constraint, PxReal constraintOffset, bool doConversion)
{
	ev4sio_Sc::DeformableVolumeSim& bSim = *core.getSim();

	PxU32 handle = mSimulationController->addSoftBodyAttachment(bSim.getLowLevelDeformableVolume(), sim.getLowLevelDeformableVolume(), tetIdx0, tetIdx1, 
		tetBarycentric0, tetBarycentric1, constraint, constraintOffset, sim.isActive() || bSim.isActive(), doConversion);

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, bSim.getNodeIndex(), ev4sio_IG::Edge::eSOFT_BODY_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::removeSoftBodyAttachment(DeformableVolumeCore& core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 handle)
{
	ev4sio_Sc::DeformableVolumeSim& bSim = *core.getSim();
	mSimulationController->removeSoftBodyAttachment(bSim.getLowLevelDeformableVolume(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, bSim.getNodeIndex());
}

void ev4sio_Sc::Scene::addClothFilter(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 triIdx, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx)
{
	ev4sio_Sc::DeformableSurfaceSim& bSim = *core.getSim();

	mSimulationController->addClothFilter(sim.getLowLevelDeformableVolume(), bSim.getLowLevelDeformableSurface(), triIdx,tetIdx);
}

void ev4sio_Sc::Scene::removeClothFilter(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 triIdx, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx)
{
	ev4sio_Sc::DeformableSurfaceSim& bSim = *core.getSim();
	mSimulationController->removeClothFilter(sim.getLowLevelDeformableVolume(), bSim.getLowLevelDeformableSurface(), triIdx, tetIdx);
}

PxU32 ev4sio_Sc::Scene::addClothAttachment(ev4sio_Sc::DeformableSurfaceCore& core, PxU32 triIdx, const PxVec4& triBarycentric, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 tetIdx, 
	const PxVec4& tetBarycentric, PxConeLimitedConstraint* constraint, PxReal constraintOffset, bool doConversion)
{
	ev4sio_Sc::DeformableSurfaceSim& bSim = *core.getSim();

	PxU32 handle = mSimulationController->addClothAttachment(sim.getLowLevelDeformableVolume(), bSim.getLowLevelDeformableSurface(), triIdx, triBarycentric,
		tetIdx, tetBarycentric, constraint, constraintOffset, sim.isActive(), doConversion);

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, bSim.getNodeIndex(), ev4sio_IG::Edge::eFEM_CLOTH_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::removeClothAttachment(ev4sio_Sc::DeformableSurfaceCore& core, ev4sio_Sc::DeformableVolumeSim& sim, PxU32 handle)
{
	PX_UNUSED(core);
	ev4sio_Sc::DeformableSurfaceSim& bSim = *core.getSim();
	mSimulationController->removeClothAttachment(sim.getLowLevelDeformableVolume(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, bSim.getNodeIndex());
}

PxU32 ev4sio_Sc::Scene::addRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableSurfaceSim& sim, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addRigidAttachment(sim.getLowLevelDeformableSurface(), sim.getNodeIndex(), body, nodeIndex,
		vertId, actorSpacePose, constraint, sim.isActive());

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex, ev4sio_IG::Edge::eFEM_CLOTH_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::removeRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableSurfaceSim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeRigidAttachment(sim.getLowLevelDeformableSurface(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex);
}

void ev4sio_Sc::Scene::addTriRigidFilter(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableSurfaceSim& sim, PxU32 triIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->addTriRigidFilter(sim.getLowLevelDeformableSurface(), nodeIndex, triIdx);
}

void ev4sio_Sc::Scene::removeTriRigidFilter(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableSurfaceSim& sim, PxU32 triIdx)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeTriRigidFilter(sim.getLowLevelDeformableSurface(), nodeIndex, triIdx);
}

PxU32 ev4sio_Sc::Scene::addTriRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableSurfaceSim& sim, PxU32 triIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	PxNodeIndex nodeIndex;
	PxsRigidBody* body = NULL;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
		body = &core->getSim()->getLowLevelBody();
	}

	PxU32 handle = mSimulationController->addTriRigidAttachment(sim.getLowLevelDeformableSurface(), body, nodeIndex, triIdx, barycentric, actorSpacePose, constraint, sim.isActive());

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex, ev4sio_IG::Edge::eFEM_CLOTH_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::removeTriRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::DeformableSurfaceSim& sim, PxU32 handle)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}

	mSimulationController->removeTriRigidAttachment(sim.getLowLevelDeformableSurface(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex);
}

void ev4sio_Sc::Scene::addClothFilter(DeformableSurfaceCore& core0, PxU32 triIdx0, ev4sio_Sc::DeformableSurfaceSim& sim1, PxU32 triIdx1)
{
	ev4sio_Sc::DeformableSurfaceSim& sim0 = *core0.getSim();

	mSimulationController->addClothFilter(sim0.getLowLevelDeformableSurface(), sim1.getLowLevelDeformableSurface(), triIdx0, triIdx1);
}

void ev4sio_Sc::Scene::removeClothFilter(DeformableSurfaceCore& core, PxU32 triIdx0, DeformableSurfaceSim& sim1, PxU32 triIdx1)
{
	ev4sio_Sc::DeformableSurfaceSim& sim0 = *core.getSim();
	mSimulationController->removeClothFilter(sim0.getLowLevelDeformableSurface(), sim1.getLowLevelDeformableSurface(), triIdx0, triIdx1);
}

PxU32 ev4sio_Sc::Scene::addTriClothAttachment(DeformableSurfaceCore& core, PxU32 triIdx0, const PxVec4& barycentric0, ev4sio_Sc::DeformableSurfaceSim& sim1, PxU32 triIdx1, const PxVec4& barycentric1)
{
	ev4sio_Sc::DeformableSurfaceSim& sim0 = *core.getSim();

	PxU32 handle = mSimulationController->addTriClothAttachment(sim0.getLowLevelDeformableSurface(), sim1.getLowLevelDeformableSurface(), triIdx0, triIdx1, barycentric0, barycentric1, sim1.isActive() || sim0.isActive());

	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim0, sim1.getNodeIndex(), ev4sio_IG::Edge::eFEM_CLOTH_CONTACT);

	return handle;
}

void ev4sio_Sc::Scene::removeTriClothAttachment(DeformableSurfaceCore& core, DeformableSurfaceSim& sim1, PxU32 handle)
{
	ev4sio_Sc::DeformableSurfaceSim& sim0 = *core.getSim();
	mSimulationController->removeTriClothAttachment(sim0.getLowLevelDeformableSurface(), handle);

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim0, sim1.getNodeIndex());
}

void ev4sio_Sc::Scene::addParticleSystemSimControl(ev4sio_Sc::ParticleSystemCore& core)
{
	ev4sio_Sc::ParticleSystemSim* sim = core.getSim();

	if (sim)
	{
		mSimulationController->addParticleSystem(sim->getLowLevelParticleSystem(), sim->getNodeIndex());
		
		mLLContext->getNphaseImplementationContext()->registerShape(sim->getNodeIndex(), sim->getCore().getShapeCore().getCore(), sim->getLowLevelParticleSystem()->getElementId(), sim->getPxActor());
	}
}

void ev4sio_Sc::Scene::removeParticleSystemSimControl(ev4sio_Sc::ParticleSystemCore& core)
{
	ev4sio_Sc::ParticleSystemSim* sim = core.getSim();

	if (sim)
	{
		mLLContext->getNphaseImplementationContext()->unregisterShape(sim->getCore().getShapeCore().getCore(), sim->getShapeSim().getElementID());
		mSimulationController->releaseParticleSystem(sim->getLowLevelParticleSystem());
	}
}


void ev4sio_Sc::Scene::addRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::ParticleSystemSim& sim)
{
	PxNodeIndex nodeIndex;

	if (core)
	{
		nodeIndex = core->getSim()->getNodeIndex();
	}
	
	addToIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex, ev4sio_IG::Edge::ePARTICLE_SYSTEM_CONTACT);
}

void ev4sio_Sc::Scene::removeRigidAttachment(ev4sio_Sc::BodyCore* core, ev4sio_Sc::ParticleSystemSim& sim)
{
	PxNodeIndex nodeIndex;
	if (core)
		nodeIndex = core->getSim()->getNodeIndex();

	removeFromIslandManager(mParticleOrSoftBodyRigidInteractionMap, mSimpleIslandManager, sim, nodeIndex);
}

PxActor** ev4sio_Sc::Scene::getActiveDeformableVolumeActors(PxU32& nbActorsOut)
{
	nbActorsOut = mActiveDeformableVolumeActors.size();

	if (!nbActorsOut)
		return NULL;

	return mActiveDeformableVolumeActors.begin();
}

void ev4sio_Sc::Scene::setActiveDeformableVolumeActors(PxActor** actors, PxU32 nbActors)
{
	mActiveDeformableVolumeActors.forceSize_Unsafe(0);
	mActiveDeformableVolumeActors.resize(nbActors);
	PxMemCopy(mActiveDeformableVolumeActors.begin(), actors, sizeof(PxActor*) * nbActors);
}

//PxActor** ev4sio_Sc::Scene::getActiveDeformableSurfaceActors(PxU32& nbActorsOut)
//{
//	nbActorsOut = mActiveDeformableSurfaceActors.size();
//
//	if (!nbActorsOut)
//		return NULL;
//
//	return mActiveDeformableSurfaceActors.begin();
//}
//
//void ev4sio_Sc::Scene::setActiveDeformableSurfaceActors(PxActor** actors, PxU32 nbActors)
//{
//	mActiveDeformableSurfaceActors.forceSize_Unsafe(0);
//	mActiveDeformableSurfaceActors.resize(nbActors);
//	PxMemCopy(mActiveDeformableSurfaceActors.begin(), actors, sizeof(PxActor*) * nbActors);
//}

#endif //PX_SUPPORT_GPU_PHYSX

