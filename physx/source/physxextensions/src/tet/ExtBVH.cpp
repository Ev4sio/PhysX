// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ExtBVH.h"
#include "ExtUtilities.h"

using namespace ev4sio_physx;
using namespace ev4sio_Ext;
using namespace ev4sio_Gu;

void BVHDesc::query(const PxBounds3& bounds, PxArray<PxI32>& items)
{
	items.clear();
	IntersectionCollectingTraversalController traversalController(bounds, items);
	traverseBVH(tree.begin(), traversalController, 0);
}
		
void BVHBuilder::build(BVHDesc& bvh, const PxBounds3* items, PxI32 n)
{
	AABBTreeBounds boxes;
	boxes.init(n);
	for (PxI32 i = 0; i < n; ++i)			
		boxes.getBounds()[i] = items[i];
	ev4sio_Gu::buildAABBTree(n, boxes, bvh.tree);
}
