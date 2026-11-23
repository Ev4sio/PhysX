// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxBounds3.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "GuConvexUtilsInternal.h"
#include "GuBoxConversion.h"
#include "GuConvexMesh.h"
#include "CmScaling.h"
#include "CmMatrix34.h"

using namespace ev4sio_physx;
using namespace ev4sio_Gu;
using namespace ev4sio_Cm;

void ev4sio_Gu::computeHullOBB(Box& hullOBB, const PxBounds3& hullAABB, float offset,
						const PxMat34& convexPose, 
						const PxMat34& meshPose, const FastVertex2ShapeScaling& meshScaling, bool idtScaleMesh)
{
	// transform bounds = mesh space
	const PxMat34 m0to1 = meshPose.transformTranspose(convexPose);

	hullOBB.extents = hullAABB.getExtents() + PxVec3(offset);
	hullOBB.center = m0to1.transform(hullAABB.getCenter());
	hullOBB.rot = m0to1.m;
	
	if(!idtScaleMesh)
		meshScaling.transformQueryBounds(hullOBB.center, hullOBB.extents, hullOBB.rot);
}

void ev4sio_Gu::computeVertexSpaceOBB(Box& dst, const Box& src, const PxTransform& meshPose, const PxMeshScale& meshScale)
{
	// AP scaffold failure in x64 debug in GuConvexUtilsInternal.cpp
	//PX_ASSERT("Performance warning - this path shouldn't execute for identity mesh scale." && !meshScale.isIdentity());

	dst = transform(meshScale.getInverse() * Matrix34FromTransform(meshPose.getInverse()), src);
}

void ev4sio_Gu::computeOBBAroundConvex(Box& obb, const PxConvexMeshGeometry& convexGeom, const PxConvexMesh* cm, const PxTransform& convexPose)
{
	const CenterExtents& aabb = static_cast<const ev4sio_Gu::ConvexMesh*>(cm)->getLocalBoundsFast();

	if(convexGeom.scale.isIdentity())
	{
		const PxMat33Padded m(convexPose.q);
		obb = ev4sio_Gu::Box(m.transform(aabb.mCenter) + convexPose.p, aabb.mExtents, m);
	}
	else
	{
		obb = transform(Matrix34FromTransform(convexPose) * toMat33(convexGeom.scale), Box(aabb.mCenter, aabb.mExtents, PxMat33(PxIdentity)));
	}
}
