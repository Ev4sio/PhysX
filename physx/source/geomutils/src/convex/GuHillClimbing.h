// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_HILL_CLIMBING_H
#define GU_HILL_CLIMBING_H

#include "common/PxPhysXCommonConfig.h"

namespace ev4sio_physx
{
	namespace ev4sio_Gu
	{
		struct BigConvexRawData;
	}

	void localSearch(PxU32& id, const PxVec3& dir, const PxVec3* verts, const ev4sio_Gu::BigConvexRawData* val);
}

#endif
