// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxTransform.h"
#include "NpBounds.h"
#include "CmTransformUtils.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "GuBounds.h"

using namespace ev4sio_physx;
using namespace ev4sio_Sq;

static void computeStaticWorldAABB(PxBounds3& bounds, const NpShape& npShape, const NpActor& npActor)
{
	const PxTransform& shape2Actor = npShape.getCore().getShape2Actor();

	PX_ALIGN(16, PxTransform) globalPose;

	ev4sio_Cm::getStaticGlobalPoseAligned(static_cast<const NpRigidStatic&>(npActor).getCore().getActor2World(), shape2Actor, globalPose);

	ev4sio_Gu::computeBounds(bounds, npShape.getCore().getGeometry(), globalPose, 0.0f, SQ_PRUNER_INFLATION);
}

static void computeDynamicWorldAABB(PxBounds3& bounds, const NpShape& npShape, const NpActor& npActor)
{
	const PxTransform& shape2Actor = npShape.getCore().getShape2Actor();

	PX_ALIGN(16, PxTransform) globalPose;
	{
		PX_ALIGN(16, PxTransform) kinematicTarget;
		const PxU16 sqktFlags = PxRigidBodyFlag::eKINEMATIC | PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES;
		// PT: TODO: revisit this once the dust has settled
		PX_ASSERT(npActor.getNpType()==NpType::eBODY_FROM_ARTICULATION_LINK || npActor.getNpType()==NpType::eBODY);
		const ev4sio_Sc::BodyCore& core = npActor.getNpType()==NpType::eBODY_FROM_ARTICULATION_LINK ? static_cast<const NpArticulationLink&>(npActor).getCore() : static_cast<const NpRigidDynamic&>(npActor).getCore();
		const bool useTarget = (PxU16(core.getFlags()) & sqktFlags) == sqktFlags;
		const PxTransform& body2World = (useTarget && core.getKinematicTarget(kinematicTarget)) ? kinematicTarget : core.getBody2World();
		ev4sio_Cm::getDynamicGlobalPoseAligned(body2World, shape2Actor, core.getBody2Actor(), globalPose);
	}

	ev4sio_Gu::computeBounds(bounds, npShape.getCore().getGeometry(), globalPose, 0.0f, SQ_PRUNER_INFLATION);
}

const ComputeBoundsFunc ev4sio_Sq::gComputeBoundsTable[2] =
{ 
	computeStaticWorldAABB, 
	computeDynamicWorldAABB
};
