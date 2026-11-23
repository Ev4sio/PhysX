// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "PxPhysicsAPI.h"


namespace snippetvehicle
{

ev4sio_physx::PxFilterFlags VehicleFilterShader(
	ev4sio_physx::PxFilterObjectAttributes attributes0, ev4sio_physx::PxFilterData filterData0,
	ev4sio_physx::PxFilterObjectAttributes attributes1, ev4sio_physx::PxFilterData filterData1,
	ev4sio_physx::PxPairFlags& pairFlags, const void* constantBlock, ev4sio_physx::PxU32 constantBlockSize);


bool parseVehicleDataPath(int argc, const char *const* argv, const char* snippetName,
	const char*& vehicleDataPath);

}//namespace snippetvehicle
