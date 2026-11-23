// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxIntrinsics.h"
#include "GuInternal.h"
#include "GuBox.h"
#include "GuCapsule.h"

using namespace ev4sio_physx;

/**
*	Computes an OBB surrounding the capsule.
*	\param		box		[out] the OBB
*/
void ev4sio_Gu::computeBoxAroundCapsule(const ev4sio_Gu::Capsule& capsule, ev4sio_Gu::Box& box)
{
	// Box center = center of the two capsule's endpoints
	box.center = capsule.computeCenter();

	// Box extents
	const PxF32 d = (capsule.p0 - capsule.p1).magnitude();
	box.extents.x = capsule.radius + (d * 0.5f);
	box.extents.y = capsule.radius;
	box.extents.z = capsule.radius;

	// Box orientation
	if(d==0.0f)
	{
		box.rot = PxMat33(PxIdentity);
	}
	else
	{
		PxVec3 dir, right, up;
		PxComputeBasisVectors(capsule.p0, capsule.p1, dir, right, up);
		box.setAxes(dir, right, up);
	}
}
