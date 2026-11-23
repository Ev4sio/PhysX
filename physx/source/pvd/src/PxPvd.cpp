// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPvdImpl.h"
namespace ev4sio_physx
{
namespace pvdsdk
{
ForwardingAllocator gForwardingAllocator;
PxAllocatorCallback* gPvdAllocatorCallback = &gForwardingAllocator;
} // namespace pvdsdk

PxPvd* ev4sio_PxCreatePvd(PxFoundation& foundation)
{
	pvdsdk::gPvdAllocatorCallback = &foundation.getAllocatorCallback();
	pvdsdk::PvdImpl::initialize();
	return pvdsdk::PvdImpl::getInstance();
}

} // namespace ev4sio_physx
