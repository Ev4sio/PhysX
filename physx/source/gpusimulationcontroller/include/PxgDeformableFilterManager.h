// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef PXG_DEFORMABLE_FILTER_MANAGER_H
#define PXG_DEFORMABLE_FILTER_MANAGER_H

#include "foundation/PxArray.h"
#include "foundation/PxSimpleTypes.h"
#include "CmPinnableArray.h"
#include "PxsHeapMemoryAllocator.h"
#include "PxgSimulationCoreDesc.h"

namespace ev4sio_physx
{
	namespace ev4sio_Gu
	{
		class BVTetrahedronMesh;
		class TriangleMesh;
	}

	class PxgDeformableSurfaceTopology;

	// Host staging for the element filter pairs of one deformable and one pair type, sorted so that the
	// deformable kernels can binary search them. Per pair references track how many filters expanded to a
	// pair, so two filters can share one, which is dropped once the last of them is removed. mInDirtyList
	// keeps a deformable from entering the manager's dirty list twice.
	struct PxgDeformableFilterPairs : public PxUserAllocated
	{
		PxgDeformableFilterPairs(ev4sio_Cm::VirtualAllocatorCallback& hostAlloc, int heapStatGroup)
			: mPairs(hostAlloc, heapStatGroup), mDirty(false), mInDirtyList(false)
		{}

		ev4sio_Cm::PinnableArray<PxgDeformablePairInfo> mPairs;
		PxArray<PxU32> mReferences;
		bool mDirty;
		bool mInDirtyList;
	};

	// The filter pairs of one pair type, held per deformable and keyed by its GPU remap id. An actor with
	// no filters has no array, so a kernel reading its pair count gets zero without searching anything.
	class PxgDeformableFilterManager
	{
	public:
		PxgDeformableFilterManager(ev4sio_Cm::VirtualAllocatorCallback& hostAlloc, PxsHeapStats::Enum heapStatGroup)
			: mHostAlloc(hostAlloc), mHeapStatGroup(heapStatGroup)
		{}

		~PxgDeformableFilterManager();

		// Adds or removes one element filter of the pair type this manager holds, expanding the elements it
		// references into the pairs the kernels of that pair type probe. A deformable is passed as its
		// collision mesh and its GPU remap id. Element indices are the API's and are mapped to the GPU
		// mesh numbering here.

		void updateVolumeRigid(const ev4sio_Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
			const PxNodeIndex& rigidNodeIndex, const PxU32* tetIndices, PxU32 tetIndicesSize, bool add);

		// A PxRigidStatic is keyed on its shapes rather than on a node, with one pair per shape and element.
		// The shapes are passed as their transform cache ids, see PxgEncodeStaticShapeRef.
		void updateVolumeStaticShapes(const ev4sio_Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
			const PxU32* shapeCacheIds, PxU32 nbShapes, const PxU32* tetIndices, PxU32 tetIndicesSize, bool add);

		void updateVolumeVolume(const ev4sio_Gu::BVTetrahedronMesh& volumeMesh0, PxU32 volumeId0,
			const ev4sio_Gu::BVTetrahedronMesh& volumeMesh1, PxU32 volumeId1,
			const PxU32* tetIndices0, const PxU32* tetIndices1, PxU32 tetIndicesSize, bool add);

		void updateVolumeSurface(const ev4sio_Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
			const ev4sio_Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId,
			const PxU32* triIndices, const PxU32* tetIndices, PxU32 indicesSize, bool add);

		void updateSurfaceRigid(const ev4sio_Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId,
			const PxNodeIndex& rigidNodeIndex, const PxU32* triIndices, PxU32 triIndicesSize, bool add);

		void updateSurfaceStaticShapes(const ev4sio_Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId,
			const PxU32* shapeCacheIds, PxU32 nbShapes, const PxU32* triIndices, PxU32 triIndicesSize, bool add);

		// The topologies carry the triangle edge ownership the edge-edge pairs are keyed on. The caller
		// owns them, building one as each surface enters the scene.
		void updateSurfaceSurface(const ev4sio_Gu::TriangleMesh& surfaceMesh0, const PxgDeformableSurfaceTopology& topology0,
			PxU32 surfaceId0, const ev4sio_Gu::TriangleMesh& surfaceMesh1, const PxgDeformableSurfaceTopology& topology1,
			PxU32 surfaceId1, const PxU32* triIndices0, const PxU32* triIndices1, PxU32 triIndicesSize, bool add);

		// Takes the deformable off the dirty list and frees its array. The id stays reserved until
		// endSimulation, so no other deformable can take it before the next upload, but the entry still
		// has to go because the pairs it refers to are freed here.
		void release(PxU32 deformableId);

		PX_FORCE_INLINE const PxArray<PxgDeformableFilterPairs*>& getPerActorPairs() const { return mPerActorPairs; }
		PX_FORCE_INLINE const PxArray<PxU32>& getDirtyIds() const { return mDirtyIds; }

		// True while any deformable is waiting for the next upload.
		PX_FORCE_INLINE bool isDirty() const { return !mDirtyIds.empty(); }

		PX_FORCE_INLINE void clearDirtyIds() { mDirtyIds.forceSize_Unsafe(0); }

	private:
		PX_NOCOPY(PxgDeformableFilterManager)

		// Applies the entries expanded for one deformable. Accepts them in any order.
		void update(PxArray<PxgDeformablePairInfo>& appended, PxU32 deformableId, bool add);

		// Created on first use and destroyed with the deformable.
		PxgDeformableFilterPairs& getOrCreate(PxU32 deformableId);
		void markDirty(PxU32 deformableId);

		ev4sio_Cm::VirtualAllocatorCallback& mHostAlloc;
		PxsHeapStats::Enum mHeapStatGroup;

		PxArray<PxgDeformableFilterPairs*> mPerActorPairs;
		PxArray<PxU32> mDirtyIds;
	};
}

#endif // PXG_DEFORMABLE_FILTER_MANAGER_H
