// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
		virtual		bool						addCompound(ev4sio_Gu::PrunerHandle* results, const ev4sio_Gu::BVH& bvh, PrunerCompoundId compoundId, const PxTransform& transform, bool isDynamic, const ev4sio_Gu::PrunerPayload* data, const PxTransform* transforms) PX_OVERRIDE;
		virtual		bool						removeCompound(PrunerCompoundId compoundId, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback) PX_OVERRIDE;
		virtual		bool						updateCompound(PrunerCompoundId compoundId, const PxTransform& transform) PX_OVERRIDE;
		// object level
		virtual		void						updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const ev4sio_Gu::PrunerHandle handle) PX_OVERRIDE;
		virtual		void						removeObject(PrunerCompoundId compoundId, const ev4sio_Gu::PrunerHandle handle, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback) PX_OVERRIDE;
		virtual		bool						addObject(PrunerCompoundId compoundId, ev4sio_Gu::PrunerHandle& result, const PxBounds3& bounds, const ev4sio_Gu::PrunerPayload userData, const PxTransform& transform) PX_OVERRIDE;
		//queries
		virtual		bool						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const PX_OVERRIDE;
		virtual		bool						overlap(const ev4sio_Gu::ShapeData& queryVolume, CompoundPrunerOverlapCallback&, PxCompoundPrunerQueryFlags flags) const PX_OVERRIDE;
		virtual		bool						sweep(const ev4sio_Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const PX_OVERRIDE;
		virtual		const ev4sio_Gu::PrunerPayload&	getPayloadData(ev4sio_Gu::PrunerHandle handle, PrunerCompoundId compoundId, ev4sio_Gu::PrunerPayloadData* data) const PX_OVERRIDE;
		virtual		void						preallocate(PxU32 nbEntries) PX_OVERRIDE;
		virtual		bool						setTransform(ev4sio_Gu::PrunerHandle handle, PrunerCompoundId compoundId, const PxTransform& transform) PX_OVERRIDE;
		virtual		const PxTransform&			getTransform(PrunerCompoundId compoundId)	const PX_OVERRIDE;
		virtual		void						visualizeEx(PxRenderOutput& out, PxU32 color, bool drawStatic, bool drawDynamic)	const PX_OVERRIDE;
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
