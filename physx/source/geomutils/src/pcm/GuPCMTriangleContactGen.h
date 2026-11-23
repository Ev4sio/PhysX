// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PCM_TRIANGLE_CONTACT_GEN_H
#define GU_PCM_TRIANGLE_CONTACT_GEN_H

#include "GuPCMContactGenUtil.h"
#include "GuPersistentContactManifold.h"

namespace ev4sio_physx
{
	class PxTriangleMeshGeometry;
	class PxHeightFieldGeometry;

namespace ev4sio_Gu
{
	bool PCMContactConvexMesh(const ev4sio_Gu::PolygonalData& polyData0, const ev4sio_Gu::SupportLocal* polyMap, const aos::FloatVArg minMargin, const PxBounds3& hullAABB,
						const PxTriangleMeshGeometry& shapeMesh,
						const PxTransform& transform0, const PxTransform& transform1, PxReal contactDistance, PxContactBuffer& contactBuffer,
						const ev4sio_Cm::FastVertex2ShapeScaling& convexScaling, const ev4sio_Cm::FastVertex2ShapeScaling& meshScaling,
						bool idtConvexScale, bool idtMeshScale,
						ev4sio_Gu::MultiplePersistentContactManifold& multiManifold, PxRenderOutput* renderOutput);

	bool PCMContactConvexHeightfield(const ev4sio_Gu::PolygonalData& polyData0, const ev4sio_Gu::SupportLocal* polyMap, const aos::FloatVArg minMargin, const PxBounds3& hullAABB,
						const PxHeightFieldGeometry& shapeHeightfield,
						const PxTransform& transform0, const PxTransform& transform1, PxReal contactDistance, PxContactBuffer& contactBuffer,
						const ev4sio_Cm::FastVertex2ShapeScaling& convexScaling, bool idtConvexScale,
						ev4sio_Gu::MultiplePersistentContactManifold& multiManifold, PxRenderOutput* renderOutput);
}
}

#endif
