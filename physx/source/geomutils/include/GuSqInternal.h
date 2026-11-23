// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SQ_INTERNAL_H
#define GU_SQ_INTERNAL_H

#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"

#define SQ_DEBUG_VIZ_STATIC_COLOR	PxU32(PxDebugColor::eARGB_BLUE)
#define SQ_DEBUG_VIZ_DYNAMIC_COLOR	PxU32(PxDebugColor::eARGB_RED)
#define SQ_DEBUG_VIZ_STATIC_COLOR2	PxU32(PxDebugColor::eARGB_DARKBLUE)
#define SQ_DEBUG_VIZ_DYNAMIC_COLOR2	PxU32(PxDebugColor::eARGB_DARKRED)
#define SQ_DEBUG_VIZ_COMPOUND_COLOR	PxU32(PxDebugColor::eARGB_MAGENTA)

namespace ev4sio_physx
{
	class PxRenderOutput;
	class PxBounds3;

	namespace ev4sio_Gu
	{
		class BVH;
		class AABBTree;
		class IncrementalAABBTree;
		class IncrementalAABBTreeNode;
	}

	class DebugVizCallback
	{
		public:
						DebugVizCallback()	{}
		virtual			~DebugVizCallback()	{}
		virtual	bool	visualizeNode(const ev4sio_physx::ev4sio_Gu::IncrementalAABBTreeNode& node, const ev4sio_physx::PxBounds3& bounds)	= 0;
	};
}

	PX_PHYSX_COMMON_API	void visualizeTree(ev4sio_physx::PxRenderOutput& out, ev4sio_physx::PxU32 color, const ev4sio_physx::ev4sio_Gu::BVH* tree);
	PX_PHYSX_COMMON_API	void visualizeTree(ev4sio_physx::PxRenderOutput& out, ev4sio_physx::PxU32 color, const ev4sio_physx::ev4sio_Gu::AABBTree* tree);
	PX_PHYSX_COMMON_API	void visualizeTree(ev4sio_physx::PxRenderOutput& out, ev4sio_physx::PxU32 color, const ev4sio_physx::ev4sio_Gu::IncrementalAABBTree* tree, ev4sio_physx::DebugVizCallback* cb=NULL);

	// PT: macros to try limiting the code duplication in headers. Mostly it just redefines the
	// SqPruner API in implementation classes, and you shouldn't have to worry about it.
	// Note that this assumes pool-based pruners with an mPool class member (for now).

#define DECLARE_BASE_PRUNER_API																									\
	virtual	void					shiftOrigin(const PxVec3& shift) PX_OVERRIDE;												\
	virtual	void					visualize(PxRenderOutput& out, PxU32 primaryColor, PxU32 secondaryColor) const PX_OVERRIDE;

#define DECLARE_PRUNER_API_COMMON																																																\
	virtual	bool					addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* data, const PxTransform* transforms, PxU32 count, bool hasPruningStructure) PX_OVERRIDE;					\
	virtual	void					removeObjects(const PrunerHandle* handles, PxU32 count, PrunerPayloadRemovalCallback* removalCallback) PX_OVERRIDE;																			\
	virtual void					updateObjects(const PrunerHandle* handles, PxU32 count, float inflation, const PxU32* boundsIndices, const PxBounds3* newBounds, const PxTransform32* newTransforms) PX_OVERRIDE;			\
	virtual	void					purge() PX_OVERRIDE;																																										\
	virtual	void					commit() PX_OVERRIDE;																																										\
	virtual	void					merge(const void* mergeParams) PX_OVERRIDE;																																					\
	virtual	bool					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, ev4sio_Gu::PrunerRaycastCallback&)				const	PX_OVERRIDE;													\
	virtual	bool					overlap(const ev4sio_Gu::ShapeData& queryVolume, ev4sio_Gu::PrunerOverlapCallback&)												const	PX_OVERRIDE;													\
	virtual	bool					sweep(const ev4sio_Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, ev4sio_Gu::PrunerRaycastCallback&)	const	PX_OVERRIDE;													\
	virtual	const PrunerPayload&	getPayloadData(PrunerHandle handle, PrunerPayloadData* data)														const	PX_OVERRIDE { return mPool.getPayloadData(handle, data);	}	\
	virtual	void					preallocate(PxU32 entries)																									PX_OVERRIDE { mPool.preallocate(entries);					}	\
	virtual	bool					setTransform(PrunerHandle handle, const PxTransform& transform) PX_OVERRIDE { return mPool.setTransform(handle, transform);	}	\
	virtual	void					getGlobalBounds(PxBounds3&)																							const	PX_OVERRIDE;

#endif
