// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <PxPhysicsAPI.h>

#include <functional>
#include <unordered_set>

namespace omni
{
namespace ev4sio_physx
{
inline void* copyAlignedMemory(const void* inMemory, size_t memSize)
{
    void* nm = malloc(memSize + PX_SERIAL_FILE_ALIGN);
    const void* memoryB = (const void*)((size_t(inMemory) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
    void* memoryA = (void*)((size_t(nm) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
    memcpy(memoryA, memoryB, memSize);

    return nm;
}

void mirrorActor(::ev4sio_physx::PxActor& actor,
                 void*& outMemory,
                 uint32_t& memSize,
                 ::ev4sio_physx::PxSerializationRegistry& sr,
                 ::ev4sio_physx::PxCollection*& outSharedCol);
::ev4sio_physx::PxActor* instantiateMirrorActor(void* nonalignedBlock,
                                         ::ev4sio_physx::PxSerializationRegistry& sr,
                                         ::ev4sio_physx::PxScene& scene,
                                         ::ev4sio_physx::PxCollection*& collectionOut,
                                         const ::ev4sio_physx::PxCollection* sharedCollection);

void mirrorHierarchy(::ev4sio_physx::PxScene& scene,
                     void*& outMemory,
                     uint32_t& memSize,
                     ::ev4sio_physx::PxSerializationRegistry& sr,
                     ::ev4sio_physx::PxCollection*& outSharedCol,
                     const std::unordered_set<void*>& inPtrs);

using ProcessObjectFn = std::function<void(::ev4sio_physx::PxBase& object)>;

::ev4sio_physx::PxCollection* instantiateHierarchy(void* nonalignedBlock,
                                            ::ev4sio_physx::PxSerializationRegistry& sr,
                                            ::ev4sio_physx::PxScene& scene,
                                            const ::ev4sio_physx::PxCollection* sharedCollection,
                                            const ProcessObjectFn& proccesObjectFn);
} // namespace ev4sio_physx
} // namespace omni
