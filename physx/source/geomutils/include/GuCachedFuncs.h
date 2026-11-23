// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CACHED_FUNCS_H
#define GU_CACHED_FUNCS_H

#include "GuRaycastTests.h"
#include "GuSweepTests.h"
#include "GuOverlapTests.h"

namespace ev4sio_physx
{
namespace ev4sio_Gu
{
	struct CachedFuncs
	{
		CachedFuncs() :
			mCachedRaycastFuncs	(ev4sio_Gu::getRaycastFuncTable()),
			mCachedSweepFuncs	(ev4sio_Gu::getSweepFuncTable()),
			mCachedOverlapFuncs	(ev4sio_Gu::getOverlapFuncTable())
		{
		}

		const ev4sio_Gu::GeomRaycastTable&	mCachedRaycastFuncs;
		const ev4sio_Gu::GeomSweepFuncs&	mCachedSweepFuncs;
		const ev4sio_Gu::GeomOverlapTable*	mCachedOverlapFuncs;

		PX_NOCOPY(CachedFuncs)
	};
}
}

#endif
