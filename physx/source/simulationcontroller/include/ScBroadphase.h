// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_BROADPHASE_H
#define SC_BROADPHASE_H

#include "PxPhysXConfig.h"
#include "foundation/PxArray.h"

// PT: this class captures parts of the ev4sio_Sc::Scene that deals with broadphase matters.

namespace ev4sio_physx
{
	class PxBroadPhaseCallback;

namespace ev4sio_Bp
{
	class AABBManagerBase;
}

namespace ev4sio_Sc
{
	class ObjectIDTracker;

	class BroadphaseManager
	{
		public:
													BroadphaseManager();
													~BroadphaseManager();

			PX_FORCE_INLINE	void					setBroadPhaseCallback(PxBroadPhaseCallback* callback)	{ mBroadPhaseCallback = callback;	}
			PX_FORCE_INLINE	PxBroadPhaseCallback*	getBroadPhaseCallback()	const							{ return mBroadPhaseCallback;		}

							void					prepareOutOfBoundsCallbacks(ev4sio_Bp::AABBManagerBase* aabbManager);
							bool					fireOutOfBoundsCallbacks(ev4sio_Bp::AABBManagerBase* aabbManager, const ObjectIDTracker& tracker, PxU64 contextID);

							void					flush(ev4sio_Bp::AABBManagerBase* aabbManager);

							PxBroadPhaseCallback*	mBroadPhaseCallback;
							PxArray<PxU32>			mOutOfBoundsIDs;
	};

}
}

#endif
