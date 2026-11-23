// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_FACTORY_H
#define GU_FACTORY_H

#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuPrunerTypedef.h"

namespace ev4sio_physx
{
namespace ev4sio_Gu
{
	class Pruner;

	PX_PHYSX_COMMON_API	ev4sio_Gu::Pruner*	createBucketPruner(PxU64 contextID);
	PX_PHYSX_COMMON_API	ev4sio_Gu::Pruner*	createAABBPruner(PxU64 contextID, bool dynamic, ev4sio_Gu::CompanionPrunerType type, ev4sio_Gu::BVHBuildStrategy buildStrategy, PxU32 nbObjectsPerNode);
	PX_PHYSX_COMMON_API	ev4sio_Gu::Pruner*	createIncrementalPruner(PxU64 contextID);
}
}

#endif
