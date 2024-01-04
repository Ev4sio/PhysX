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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_SOFT_BODY_CORE_H
#define SC_SOFT_BODY_CORE_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxSoftBody.h"
#include "DySoftBodyCore.h"
#include "foundation/PxAssert.h"
#include "ScActorCore.h"
#include "ScShapeCore.h"
#include "PxFiltering.h"
#include "ScRigidCore.h" //KS - needed for ShapeChangeNotifyFlags. Move to a shared header

namespace ev4sio_physx
{
	namespace ev4sio_Sc
	{
		class SoftBodySim;
		class BodyCore;
		class FEMClothCore;
		class ParticleSystemCore;

		class SoftBodyCore : public ActorCore
		{
			// PX_SERIALIZATION
		public:
			SoftBodyCore(const PxEMPTY) : ActorCore(PxEmpty){}
			static		void			getBinaryMetaData(PxOutputStream& stream);
			//~PX_SERIALIZATION
			SoftBodyCore();
			~SoftBodyCore();

			//---------------------------------------------------------------------------------
			// External API
			//---------------------------------------------------------------------------------

			void						setMaterial(const PxU16 handle);
			void						clearMaterials();
		
			
			PxFEMParameters				getParameter() const;
			void						setParameter(const PxFEMParameters paramters);
			
			
			PxReal						getSleepThreshold() const;
			void						setSleepThreshold(const PxReal v);

			PxReal						getFreezeThreshold() const;
			void						setFreezeThreshold(const PxReal v);

			PxU16						getSolverIterationCounts() const { return mCore.solverIterationCounts; }
			void						setSolverIterationCounts(PxU16 c);

			PxReal						getWakeCounter() const;
			void						setWakeCounter(const PxReal v);
			void						setWakeCounterInternal(const PxReal v);

			bool						isSleeping() const;
			void						wakeUp(PxReal wakeCounter);
			void						putToSleep();

			PxActor*					getPxActor() const;	
	
			void						attachShapeCore(ShapeCore* shapeCore);
			void						attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxSoftBodyAuxData* simulationState);

			void						addParticleFilter(ev4sio_Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId);
			void						removeParticleFilter(ev4sio_Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId);

			PxU32						addParticleAttachment(ev4sio_Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric);
			void						removeParticleAttachment(ev4sio_Sc::ParticleSystemCore* core, PxU32 handle);

			void						addRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 vertId);
			void						removeRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 vertId);

			PxU32						addRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
			void						removeRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 handle);

			void						addTetRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 tetIdx);
			void						removeTetRigidFilter(ev4sio_Sc::BodyCore* core, PxU32 tetIdx);

			PxU32						addTetRigidAttachment(ev4sio_Sc::BodyCore* core, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose,
										PxConeLimitedConstraint* constraint);

			void						addSoftBodyFilter(ev4sio_Sc::SoftBodyCore& core, PxU32 tetIdx0, PxU32 tetIdx1);
			void						removeSoftBodyFilter(ev4sio_Sc::SoftBodyCore& core, PxU32 tetIdx0, PxU32 tetIdx1);
			void						addSoftBodyFilters(ev4sio_Sc::SoftBodyCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
			void						removeSoftBodyFilters(ev4sio_Sc::SoftBodyCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

			PxU32						addSoftBodyAttachment(ev4sio_Sc::SoftBodyCore& core, PxU32 tetIdx0, const PxVec4& triBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
										PxConeLimitedConstraint* constraint, PxReal constraintOffset);
			void						removeSoftBodyAttachment(ev4sio_Sc::SoftBodyCore& core, PxU32 handle);

			void						addClothFilter(ev4sio_Sc::FEMClothCore& core, PxU32 triIdx, PxU32 tetIdx);
			void						removeClothFilter(ev4sio_Sc::FEMClothCore& core, PxU32 triIdx, PxU32 tetIdx);

			void						addVertClothFilter(ev4sio_Sc::FEMClothCore& core, PxU32 vertIdx, PxU32 tetIdx);
			void						removeVertClothFilter(ev4sio_Sc::FEMClothCore& core, PxU32 vertIdx, PxU32 tetIdx);

			PxU32						addClothAttachment(ev4sio_Sc::FEMClothCore& core, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric, 
										PxConeLimitedConstraint* constraint, PxReal constraintOffset);
			void						removeClothAttachment(ev4sio_Sc::FEMClothCore& core,  PxU32 handle);

			PxU32						getGpuSoftBodyIndex()	const;

			void						setKinematicTargets(const PxVec4* positions, PxSoftBodyFlags flags);
			//---------------------------------------------------------------------------------
			// Internal API
			//---------------------------------------------------------------------------------
		public:
			
			SoftBodySim*		getSim() const;

			PX_FORCE_INLINE	const ev4sio_Dy::SoftBodyCore&	getCore() const { return mCore; }

			PX_FORCE_INLINE	ev4sio_Dy::SoftBodyCore& getCore() { return mCore; }

			void								setSimulationFilterData(const PxFilterData& data);
		
			PxFilterData						getSimulationFilterData() const;

			PxSoftBodyFlags						getFlags() const { return mCore.mFlags; }

			void								setFlags(PxSoftBodyFlags flags);

			PX_FORCE_INLINE PxReal				getMaxPenetrationBias() const { return mCore.maxPenBias; }
			PX_FORCE_INLINE void				setMaxPenetrationBias(PxReal p) { mCore.maxPenBias = p; }

			PX_FORCE_INLINE	PxU64&				getGpuMemStat() { return mGpuMemStat; }

			void								onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags);

		private:
			ev4sio_Dy::SoftBodyCore					mCore;
			PxFilterData						mFilterData;
			PxU64								mGpuMemStat;
		};

	} // namespace ev4sio_Sc
}
#endif

#endif
