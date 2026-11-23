// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BV_CONSTANTS_H
#define GU_BV_CONSTANTS_H

#include "foundation/PxVecMath.h"

namespace
{
	const ev4sio_physx::aos::VecU32V signMask = ev4sio_physx::aos::U4LoadXYZW((ev4sio_physx::PxU32(1)<<31), (ev4sio_physx::PxU32(1)<<31), (ev4sio_physx::PxU32(1)<<31), (ev4sio_physx::PxU32(1)<<31));
	const ev4sio_physx::aos::Vec4V epsFloat4 = ev4sio_physx::aos::V4Load(1e-9f);
	const ev4sio_physx::aos::Vec4V zeroes = ev4sio_physx::aos::V4Zero();
	const ev4sio_physx::aos::Vec4V twos = ev4sio_physx::aos::V4Load(2.0f);
	const ev4sio_physx::aos::Vec4V epsInflateFloat4 = ev4sio_physx::aos::V4Load(1e-7f);
}

#endif // GU_BV_CONSTANTS_H
