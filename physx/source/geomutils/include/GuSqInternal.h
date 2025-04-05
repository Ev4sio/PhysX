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

#define DECLARE_BASE_PRUNER_API																																	\
	virtual	void					shiftOrigin(const PxVec3& shift);																							\
	virtual	void					visualize(PxRenderOutput& out, PxU32 primaryColor, PxU32 secondaryColor) const;

#define DECLARE_PRUNER_API_COMMON																																													\
	virtual	bool					addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* data, const PxTransform* transforms, PxU32 count, bool hasPruningStructure);					\
	virtual	void					removeObjects(const PrunerHandle* handles, PxU32 count, PrunerPayloadRemovalCallback* removalCallback);																			\
	virtual void					updateObjects(const PrunerHandle* handles, PxU32 count, float inflation, const PxU32* boundsIndices, const PxBounds3* newBounds, const PxTransform32* newTransforms);			\
	virtual	void					purge();																																										\
	virtual	void					commit();																																										\
	virtual	void					merge(const void* mergeParams);																																					\
	virtual	bool					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, ev4sio_Gu::PrunerRaycastCallback&)				const;														\
	virtual	bool					overlap(const ev4sio_Gu::ShapeData& queryVolume, ev4sio_Gu::PrunerOverlapCallback&)												const;														\
	virtual	bool					sweep(const ev4sio_Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, ev4sio_Gu::PrunerRaycastCallback&)	const;														\
	virtual	const PrunerPayload&	getPayloadData(PrunerHandle handle, PrunerPayloadData* data)														const	{ return mPool.getPayloadData(handle, data);	}	\
	virtual	void					preallocate(PxU32 entries)																									{ mPool.preallocate(entries);					}	\
	virtual	bool					setTransform(PrunerHandle handle, const PxTransform& transform)																{ return mPool.setTransform(handle, transform);	}	\
	virtual	void					getGlobalBounds(PxBounds3&)																							const;

#endif
