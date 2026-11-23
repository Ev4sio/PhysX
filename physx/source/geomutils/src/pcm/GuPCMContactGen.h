// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PCM_CONTACT_GEN_H
#define GU_PCM_CONTACT_GEN_H

#include "GuConvexSupportTable.h"
#include "GuPersistentContactManifold.h"
#include "GuShapeConvex.h"
#include "GuSeparatingAxes.h"
#include "GuGJKType.h"
#include "GuGJKUtil.h"

namespace ev4sio_physx
{
namespace ev4sio_Gu
{
	//full contact gen code for box/convexhull vs convexhull
	bool generateFullContactManifold(const ev4sio_Gu::PolygonalData& polyData0, const ev4sio_Gu::PolygonalData& polyData1, const ev4sio_Gu::SupportLocal* map0, const ev4sio_Gu::SupportLocal* map1, ev4sio_Gu::PersistentContact* manifoldContacts,
		PxU32& numContacts, const aos::FloatVArg contactDist, const aos::Vec3VArg normal, const aos::Vec3VArg closestA, const aos::Vec3VArg closestB, 
		PxReal toleranceA, PxReal toleranceB, bool doOverlapTest, PxRenderOutput* renderOutput, PxReal toleranceLength);

	//full contact gen code for capsule vs convexhulll
	bool generateFullContactManifold(const ev4sio_Gu::CapsuleV& capsule, const ev4sio_Gu::PolygonalData& polyData, const ev4sio_Gu::SupportLocal* map, const aos::PxMatTransformV& aToB, ev4sio_Gu::PersistentContact* manifoldContacts,
		PxU32& numContacts, const aos::FloatVArg contactDist, aos::Vec3V& normal, const aos::Vec3VArg closest, PxReal tolerance, bool doOverlapTest, PxReal toleranceScale);

	//full contact gen code for capsule vs box
	bool generateCapsuleBoxFullContactManifold(const ev4sio_Gu::CapsuleV& capsule, const ev4sio_Gu::PolygonalData& polyData, const ev4sio_Gu::SupportLocal* map, const aos::PxMatTransformV& aToB, ev4sio_Gu::PersistentContact* manifoldContacts, PxU32& numContacts,
		const aos::FloatVArg contactDist, aos::Vec3V& normal, const aos::Vec3VArg closest, PxReal boxMargin, bool doOverlapTest, PxReal toleranceScale, PxRenderOutput* renderOutput);

	//based on the gjk status to decide whether we should do full contact gen with GJK/EPA normal. Also, this method store
	//GJK/EPA point to the manifold in case full contact gen doesn't generate any contact
	bool addGJKEPAContacts(const ev4sio_Gu::GjkConvex* relativeConvex, const ev4sio_Gu::GjkConvex* localConvex, const aos::PxMatTransformV& aToB, ev4sio_Gu::GjkStatus status,
		ev4sio_Gu::PersistentContact* manifoldContacts, const aos::FloatV replaceBreakingThreshold, const aos::FloatV toleranceLength, GjkOutput& output,
		ev4sio_Gu::PersistentContactManifold& manifold);

	//MTD code for box/convexhull vs box/convexhull
	bool computeMTD(const ev4sio_Gu::PolygonalData& polyData0, const ev4sio_Gu::PolygonalData& polyData1, const SupportLocal* map0, const SupportLocal* map1, aos::FloatV& penDepth, aos::Vec3V& normal);
	
	//MTD code for capsule vs box/convexhull
	bool computeMTD(const ev4sio_Gu::CapsuleV& capsule, const ev4sio_Gu::PolygonalData& polyData, const ev4sio_Gu::SupportLocal* map, aos::FloatV& penDepth, aos::Vec3V& normal);

	//full contact gen code for sphere vs convexhull
	bool generateSphereFullContactManifold(const ev4sio_Gu::CapsuleV& capsule, const ev4sio_Gu::PolygonalData& polyData, const ev4sio_Gu::SupportLocal* map, ev4sio_Gu::PersistentContact* manifoldContacts, PxU32& numContacts,
		const aos::FloatVArg contactDist, aos::Vec3V& normal, bool doOverlapTest);
}
}

#endif
