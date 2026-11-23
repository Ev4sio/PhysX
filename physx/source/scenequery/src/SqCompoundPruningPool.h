// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_COMPOUND_PRUNING_POOL_H
#define SQ_COMPOUND_PRUNING_POOL_H

#include "SqPruner.h"
#include "foundation/PxArray.h"
#include "GuPrunerMergeData.h"
#include "GuIncrementalAABBTree.h"
#include "GuAABBTreeBounds.h"

namespace ev4sio_physx
{
namespace ev4sio_Gu
{
	class PruningPool;
}

namespace ev4sio_Sq
{

	///////////////////////////////////////////////////////////////////////////////////////////////

	typedef PxArray<ev4sio_Gu::IncrementalAABBTreeNode*>	UpdateMap;

	///////////////////////////////////////////////////////////////////////////////////////////////

	class CompoundTree
	{
	public:
		void	updateObjectAfterManualBoundsUpdates(ev4sio_Gu::PrunerHandle handle);
		void	removeObject(ev4sio_Gu::PrunerHandle handle, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback);
		bool	addObject(ev4sio_Gu::PrunerHandle& result, const PxBounds3& bounds, const ev4sio_Gu::PrunerPayload& data, const PxTransform& transform);

	private:
		void	updateMapping(const ev4sio_Gu::PoolIndex poolIndex, ev4sio_Gu::IncrementalAABBTreeNode* node, const ev4sio_Gu::NodeList& changedLeaves);

	public:
		ev4sio_Gu::IncrementalAABBTree*	mTree;
		ev4sio_Gu::PruningPool*			mPruningPool;
		UpdateMap*					mUpdateMap;
		PxTransform					mGlobalPose;
		PxCompoundPrunerQueryFlags	mFlags;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////

	class CompoundTreePool
	{
	public:
													CompoundTreePool(PxU64 contextID);
													~CompoundTreePool();

						void						preallocate(PxU32 newCapacity);

						ev4sio_Gu::PoolIndex				addCompound(ev4sio_Gu::PrunerHandle* results, const ev4sio_Gu::BVH& bvh, const PxBounds3& compoundBounds, const PxTransform& transform, bool isDynamic, const ev4sio_Gu::PrunerPayload* data, const PxTransform* transforms);
						ev4sio_Gu::PoolIndex				removeCompound(ev4sio_Gu::PoolIndex index, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback);

						void						shiftOrigin(const PxVec3& shift);

		PX_FORCE_INLINE const ev4sio_Gu::AABBTreeBounds&	getCurrentAABBTreeBounds()	const	{ return mCompoundBounds;				}
		PX_FORCE_INLINE const PxBounds3*			getCurrentCompoundBounds()	const	{ return mCompoundBounds.getBounds();	}
		PX_FORCE_INLINE PxBounds3*					getCurrentCompoundBounds()			{ return mCompoundBounds.getBounds();	}

		PX_FORCE_INLINE const CompoundTree*			getCompoundTrees()			const	{ return mCompoundTrees;				}
		PX_FORCE_INLINE CompoundTree*				getCompoundTrees()					{ return mCompoundTrees;				}

		PX_FORCE_INLINE PxU32						getNbObjects()				const	{ return mNbObjects;					}

	private:
						bool						resize(PxU32 newCapacity);

						PxU32						mNbObjects;			//!< Current number of objects
						PxU32						mMaxNbObjects;		//!< Max. number of objects (capacity for mWorldBoxes, mObjects)

		//!< these arrays are parallel
						ev4sio_Gu::AABBTreeBounds			mCompoundBounds;	//!< List of compound world boxes, stores mNbObjects, capacity=mMaxNbObjects
						CompoundTree*				mCompoundTrees;

						PxU64						mContextID;
	};

}
}

#endif
