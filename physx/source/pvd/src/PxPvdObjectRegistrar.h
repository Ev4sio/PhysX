// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_OBJECT_REGISTRAR_H
#define PX_PVD_OBJECT_REGISTRAR_H


#include "foundation/PxHashMap.h"
#include "foundation/PxMutex.h"

#if !PX_DOXYGEN
namespace ev4sio_physx
{
namespace pvdsdk
{
#endif
class ObjectRegistrar
{
	PX_NOCOPY(ObjectRegistrar)
  public:
	ObjectRegistrar()
	{
	}
	virtual ~ObjectRegistrar()
	{
	}

	bool addItem(const void* inItem);
	bool decItem(const void* inItem);
	void clear();

  private:
	ev4sio_physx::PxHashMap<const void*, uint32_t> mRefCountMap;
	ev4sio_physx::PxMutex mRefCountMapLock;
};
#if !PX_DOXYGEN
} // pvdsdk
} // physx
#endif

#endif
