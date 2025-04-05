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

#ifndef SQ_COMPOUND_PRUNER_H
#define SQ_COMPOUND_PRUNER_H

#include "SqCompoundPruningPool.h"
#include "GuSqInternal.h"
#include "GuPrunerMergeData.h"
#include "GuIncrementalAABBTree.h"
#include "GuPruningPool.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"

namespace ev4sio_physx
{
namespace ev4sio_Sq
{
	///////////////////////////////////////////////////////////////////////////////////////////////

	typedef PxHashMap<PrunerCompoundId, ev4sio_Gu::PoolIndex>	ActorIdPoolIndexMap;
	typedef PxArray<PrunerCompoundId>					PoolIndexActorIdMap;

	///////////////////////////////////////////////////////////////////////////////////////////////

	class BVHCompoundPruner : public CompoundPruner
	{
		public:
												BVHCompoundPruner(PxU64 contextID);
		virtual									~BVHCompoundPruner();

					void						release();

		// BasePruner
												DECLARE_BASE_PRUNER_API
		//~BasePruner

		// CompoundPruner
		// compound level
		virtual		bool						addCompound(ev4sio_Gu::PrunerHandle* results, const ev4sio_Gu::BVH& bvh, PrunerCompoundId compoundId, const PxTransform& transform, bool isDynamic, const ev4sio_Gu::PrunerPayload* data, const PxTransform* transforms);
		virtual		bool						removeCompound(PrunerCompoundId compoundId, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback);
		virtual		bool						updateCompound(PrunerCompoundId compoundId, const PxTransform& transform);
		// object level
		virtual		void						updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const ev4sio_Gu::PrunerHandle handle);
		virtual		void						removeObject(PrunerCompoundId compoundId, const ev4sio_Gu::PrunerHandle handle, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback);
		virtual		bool						addObject(PrunerCompoundId compoundId, ev4sio_Gu::PrunerHandle& result, const PxBounds3& bounds, const ev4sio_Gu::PrunerPayload userData, const PxTransform& transform);
		//queries
		virtual		bool						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const;
		virtual		bool						overlap(const ev4sio_Gu::ShapeData& queryVolume, CompoundPrunerOverlapCallback&, PxCompoundPrunerQueryFlags flags) const;
		virtual		bool						sweep(const ev4sio_Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const;
		virtual		const ev4sio_Gu::PrunerPayload&	getPayloadData(ev4sio_Gu::PrunerHandle handle, PrunerCompoundId compoundId, ev4sio_Gu::PrunerPayloadData* data) const;
		virtual		void						preallocate(PxU32 nbEntries);
		virtual		bool						setTransform(ev4sio_Gu::PrunerHandle handle, PrunerCompoundId compoundId, const PxTransform& transform);
		virtual		const PxTransform&			getTransform(PrunerCompoundId compoundId)	const;
		virtual		void						visualizeEx(PxRenderOutput& out, PxU32 color, bool drawStatic, bool drawDynamic)	const;
		// ~CompoundPruner

		private:
					void						updateMapping(const ev4sio_Gu::PoolIndex poolIndex, ev4sio_Gu::IncrementalAABBTreeNode* node);
					void						updateMainTreeNode(ev4sio_Gu::PoolIndex index);

					void						test();

					ev4sio_Gu::IncrementalAABBTree		mMainTree;
					UpdateMap					mMainTreeUpdateMap;
		
					CompoundTreePool			mCompoundTreePool;
					ActorIdPoolIndexMap			mActorPoolMap;
					PoolIndexActorIdMap			mPoolActorMap;
					ev4sio_Gu::NodeList				mChangedLeaves;
		mutable		bool						mDrawStatic;
		mutable		bool						mDrawDynamic;
	};
}
}

#endif
