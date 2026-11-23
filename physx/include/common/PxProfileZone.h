// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PROFILE_ZONE_H
#define PX_PROFILE_ZONE_H

#include "foundation/PxProfiler.h"
#include "foundation/PxFoundation.h"

#if PX_DEBUG || PX_CHECKED || PX_PROFILE
	#define PX_PROFILE_ZONE(x, y)										\
		ev4sio_physx::PxProfileScoped PX_CONCAT(_scoped, __LINE__)(ev4sio_PxGetProfilerCallback(), x, false, y)
	#define PX_PROFILE_START_CROSSTHREAD(x, y)							\
		if(ev4sio_PxGetProfilerCallback())										\
			ev4sio_PxGetProfilerCallback()->zoneStart(x, true, y)
	#define PX_PROFILE_STOP_CROSSTHREAD(x, y)							\
		if(ev4sio_PxGetProfilerCallback())										\
			ev4sio_PxGetProfilerCallback()->zoneEnd(NULL, x, true, y)
	#define PX_PROFILE_VALUE(x, y, z)									\
		if(ev4sio_PxGetProfilerCallback())										\
			ev4sio_PxGetProfilerCallback()->recordData(x, y, z)
	#define PX_PROFILE_FRAME(x, y)                                                                                                         \
		if(ev4sio_PxGetProfilerCallback())                                                                                                        \
			ev4sio_PxGetProfilerCallback()->recordFrame(x, y)
#else
	#define PX_PROFILE_ZONE(x, y)
	#define PX_PROFILE_START_CROSSTHREAD(x, y)
	#define PX_PROFILE_STOP_CROSSTHREAD(x, y)
	#define PX_PROFILE_VALUE(x, y, z)
	#define PX_PROFILE_FRAME(x, y)
#endif

#endif
