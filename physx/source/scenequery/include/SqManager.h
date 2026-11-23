// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_MANAGER_H
#define SQ_MANAGER_H

// PT: SQ-API LEVEL 2 (Level 1 = SqPruner.h)
// PT: this file is part of a "high-level" set of files within Sq. The SqPruner API doesn't rely on them.
// PT: this should really be at Np level but moving it to Sq allows us to share it.

#include "common/PxPhysXCommonConfig.h"

#include "foundation/PxBitMap.h"
#include "foundation/PxArray.h"
#include "SqPruner.h"
#include "geometry/PxGeometryHelpers.h"

namespace ev4sio_physx
{
namespace ev4sio_Sq
{
	// PrunerManager-level adapter
	class Adapter
	{
		public:
						Adapter()	{}
		virtual			~Adapter()	{}

		// Retrieves the PxGeometry associated with a given PrunerPayload. This will be called by
		// the PrunerManager class when computing bounds.
		virtual	const PxGeometry&	getGeometry(const ev4sio_Gu::PrunerPayload& payload)	const	= 0;
	};

	// PT: extended pruner structure. We might want to move the additional data to the pruner itself later.
	struct PrunerExt : public PxUserAllocated
	{
//		private:
												PrunerExt();
												~PrunerExt();

						void					init(ev4sio_Gu::Pruner* pruner);
						void					flushMemory();
						void					preallocate(PxU32 nbShapes);

						void					addToDirtyList(ev4sio_Gu::PrunerHandle handle, bool dynamic, const PxTransform& transform);
						void					removeFromDirtyList(ev4sio_Gu::PrunerHandle handle);
						bool					processDirtyList(PxU32 index, const Adapter& adapter, float inflation);
//						void					growDirtyList(ev4sio_Gu::PrunerHandle handle);

		PX_FORCE_INLINE	ev4sio_Gu::Pruner*				pruner()			{ return mPruner;	}
		PX_FORCE_INLINE	const ev4sio_Gu::Pruner*		pruner()	const	{ return mPruner;	}

						ev4sio_Gu::Pruner*				mPruner;
						PxBitMap				mDirtyMap;
					PxArray<ev4sio_Gu::PrunerHandle>	mDirtyList;
						bool					mDirtyStatic;	// true if dirty list contains a static

						PX_NOCOPY(PrunerExt)

		friend class PrunerManager;
	};
}
}

#include "foundation/PxHashSet.h"
namespace ev4sio_physx
{
namespace ev4sio_Sq
{
	class CompoundPruner;
	typedef PxPair<PrunerCompoundId, ev4sio_Gu::PrunerHandle>	CompoundPair;
	typedef PxCoalescedHashSet<CompoundPair >			CompoundPrunerSet;
	// AB: extended compound pruner structure, buffers compound shape changes and flushes them.
	struct CompoundPrunerExt : public PxUserAllocated
	{
//		private:
												CompoundPrunerExt();
												~CompoundPrunerExt();

						void					flushMemory();
						void					preallocate(PxU32 nbShapes);
						void					flushShapes(const Adapter& adapter, float inflation);
						void					addToDirtyList(PrunerCompoundId compoundId, ev4sio_Gu::PrunerHandle handle, const PxTransform& transform);
						void					removeFromDirtyList(PrunerCompoundId compoundId, ev4sio_Gu::PrunerHandle handle);

		PX_FORCE_INLINE	const CompoundPruner*	pruner()	const	{ return mPruner;	}
		PX_FORCE_INLINE	CompoundPruner*			pruner()			{ return mPruner;	}

						CompoundPruner*			mPruner;						
						CompoundPrunerSet		mDirtyList;

						PX_NOCOPY(CompoundPrunerExt)

		friend class PrunerManager;
	};
}
}

#include "foundation/PxMutex.h"
#include "SqPrunerData.h"

namespace ev4sio_physx
{
class PxRenderOutput;
class PxBVH;
class PxSceneLimits;	// PT: TODO: decouple from PxSceneLimits

namespace ev4sio_Sq
{
	class PrunerManager : public PxUserAllocated
	{
	public:
														PrunerManager(PxU64 contextID, ev4sio_Gu::Pruner* staticPruner, ev4sio_Gu::Pruner* dynamicPruner,
															PxU32 dynamicTreeRebuildRateHint, float inflation,
															const PxSceneLimits& limits, const Adapter& adapter);
														~PrunerManager();

						PrunerData						addPrunerShape(const ev4sio_Gu::PrunerPayload& payload, bool dynamic, PrunerCompoundId compoundId, const PxBounds3& bounds, const PxTransform& transform, bool hasPruningStructure=false);
						void							addCompoundShape(const PxBVH& bvh, PrunerCompoundId compoundId, const PxTransform& compoundTransform, PrunerData* prunerData, const ev4sio_Gu::PrunerPayload* payloads, const PxTransform* transforms, bool isDynamic);
						void							markForUpdate(PrunerCompoundId compoundId, PrunerData s, const PxTransform& transform);
						void							removePrunerShape(PrunerCompoundId compoundId, PrunerData shapeData, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback);

		PX_FORCE_INLINE	const ev4sio_Gu::Pruner*				getPruner(PruningIndex::Enum index)			const	{ return mPrunerExt[index].mPruner;		}
		PX_FORCE_INLINE	ev4sio_Gu::Pruner*						getPruner(PruningIndex::Enum index)					{ return mPrunerExt[index].mPruner;		}
		PX_FORCE_INLINE	const CompoundPruner*			getCompoundPruner()							const	{ return mCompoundPrunerExt.mPruner;	}
		PX_FORCE_INLINE	PxU64							getContextId()								const	{ return mContextID;					}

						void							preallocate(PxU32 prunerIndex, PxU32 nbShapes);

						void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint);
		PX_FORCE_INLINE	PxU32							getDynamicTreeRebuildRateHint()				const	{ return mRebuildRateHint;				}
						
						void							flushUpdates();
						void							forceRebuildDynamicTree(PxU32 prunerIndex);

						void							updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform);
						void							removeCompoundActor(PrunerCompoundId compoundId, ev4sio_Gu::PrunerPayloadRemovalCallback* removalCallback);

						void*							prepareSceneQueriesUpdate(PruningIndex::Enum index);
						void							sceneQueryBuildStep(void* handle);

						void							sync(const ev4sio_Gu::PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices);
						void							afterSync(bool buildStep, bool commit);
						void							shiftOrigin(const PxVec3& shift);
						void							visualize(PxU32 prunerIndex, PxRenderOutput& out)	const;

						void							flushMemory();
		PX_FORCE_INLINE PxU32							getStaticTimestamp()	const	{ return mStaticTimestamp;	}
		PX_FORCE_INLINE const Adapter&					getAdapter()			const	{ return mAdapter;			}
	private:
						const Adapter&					mAdapter;
						PrunerExt						mPrunerExt[PruningIndex::eCOUNT];
						CompoundPrunerExt				mCompoundPrunerExt;

						const PxU64						mContextID;
						PxU32							mStaticTimestamp;
						PxU32							mRebuildRateHint;
						const float						mInflation;	// SQ_PRUNER_EPSILON

						PxMutex							mSQLock;  // to make sure only one query updates the dirty pruner structure if multiple queries run in parallel

						volatile bool					mPrunerNeedsUpdating;

						void							flushShapes();
		PX_FORCE_INLINE void							invalidateStaticTimestamp()		{ mStaticTimestamp++;		}

						PX_NOCOPY(PrunerManager)
	};
}
}

#endif
