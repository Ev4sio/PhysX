// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PCM_SHAPE_CONVEX_H
#define GU_PCM_SHAPE_CONVEX_H

#include "GuConvexSupportTable.h"
#include "GuPersistentContactManifold.h"
#include "GuShapeConvex.h"

namespace ev4sio_physx
{
	class PxConvexMeshGeometry;

namespace ev4sio_Gu
{
	extern const PxU8 gPCMBoxPolygonData[24];
	
	class PCMPolygonalBox
	{
	public:
									PCMPolygonalBox(const PxVec3& halfSide);

			void					getPolygonalData(ev4sio_Gu::PolygonalData* PX_RESTRICT dst)	const;

			const PxVec3&			mHalfSide;
			PxVec3					mVertices[8];
			ev4sio_Gu::HullPolygonData		mPolygons[6];
	private:
			PCMPolygonalBox& operator=(const PCMPolygonalBox&);
	};

	void getPCMConvexData(const ev4sio_Gu::ConvexHullV& convexHull, bool idtScale, ev4sio_Gu::PolygonalData& polyData);
	bool getPCMConvexData(const PxConvexMeshGeometry& shapeConvex, ev4sio_Cm::FastVertex2ShapeScaling& scaling, PxBounds3& bounds, ev4sio_Gu::PolygonalData& polyData);
}
}

#endif
