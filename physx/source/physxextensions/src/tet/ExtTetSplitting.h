// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_TET_SPLITTING_H
#define EXT_TET_SPLITTING_H

#include "ExtVec3.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashMap.h"
#include "GuTetrahedron.h"

namespace ev4sio_physx
{
namespace ev4sio_Ext
{
	using Edge = PxPair<PxI32, PxI32>;
	using Tetrahedron = ev4sio_Gu::TetrahedronT<PxI32>;

	//Splits all edges specified in edgesToSplit. The tets are modified in place. The poitns referenced by index in the key-value pari in 
	//edgesToSplit must already pe present in the points array. This functions guarantees that the tetmesh will remain watertight.
	void PX_CALL_CONV split(PxArray<Tetrahedron>& tets, const PxArray<PxVec3d>& points, const PxHashMap<PxU64, PxI32>& edgesToSplit);
}
}

#endif

