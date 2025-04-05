// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef __CU_ATOMIC_CUH__
#define __CU_ATOMIC_CUH__

#include "cuda.h"
#include "foundation/PxVec3.h"
#include "foundation/PxSimpleTypes.h"
#include "PxgIntrinsics.h"
#include "PxgArticulation.h"

static __device__ inline void AtomicAdd(float4& a, const float4 b)
{
	atomicAdd(&a.x, b.x);
	atomicAdd(&a.y, b.y);
	atomicAdd(&a.z, b.z);
	atomicAdd(&a.w, b.w);
}

static __device__ inline void AtomicAdd(float4& a, const ev4sio_physx::PxVec3 b, const ev4sio_physx::PxReal w)
{
	atomicAdd(&a.x, b.x);
	atomicAdd(&a.y, b.y);
	atomicAdd(&a.z, b.z);
	atomicAdd(&a.w, w);
}

static __device__ inline void AtomicAdd(float4& a, const ev4sio_physx::PxVec3 b)
{
	atomicAdd(&a.x, b.x);
	atomicAdd(&a.y, b.y);
	atomicAdd(&a.z, b.z);
}


__device__ inline void AtomicAdd(float* p, ev4sio_physx::PxU32 i, const ev4sio_physx::PxReal val)
{
	atomicAdd(&p[i], val);
}

__device__ inline void AtomicAdd(float4* p, ev4sio_physx::PxU32 i, const ev4sio_physx::PxVec3& v, ev4sio_physx::PxReal w)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
	atomicAdd(&p[i].w, w);
}

__device__ inline void AtomicAdd(float4* p, ev4sio_physx::PxU32 i, const ev4sio_physx::PxVec4& v)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
	atomicAdd(&p[i].w, v.w);
}

__device__ inline void AtomicAdd(float4* p, ev4sio_physx::PxU32 i, const ev4sio_physx::PxVec3& v)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
}

__device__ inline void AtomicAdd3(float4* p, ev4sio_physx::PxU32 i, const float4& v)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
}

__device__ inline void AtomicAdd3(ev4sio_physx::PxVec3& p, const ev4sio_physx::PxVec3& v)
{
	atomicAdd(&p.x, v.x);
	atomicAdd(&p.y, v.y);
	atomicAdd(&p.z, v.z);
}

__device__ inline float AtomicMin(float* address, float val)
{
	int *address_as_int = (int*)address;
	int old = *address_as_int, assumed;

	while (val < __int_as_float(old))
	{
		assumed = old;
		old = atomicCAS(address_as_int, assumed,
			__float_as_int(val));
	}

	return __int_as_float(old);
}

inline __device__ float AtomicMax(float* address, float val)
{
	int *address_as_int = (int*)address;
	int old = *address_as_int, assumed;

	while (val > __int_as_float(old))
	{
		assumed = old;
		old = atomicCAS(address_as_int, assumed,
			__float_as_int(val));
	}

	return __int_as_float(old);
}


//Some compiler was complaining about not supporting atomicOr on 64bit integers
PX_FORCE_INLINE static __device__ void AtomicOr(ev4sio_physx::PxU64* address, const ev4sio_physx::PxU64 mask)
{
	ev4sio_physx::PxU32* address32 = reinterpret_cast<ev4sio_physx::PxU32*>(address);
	const ev4sio_physx::PxU32* maskPtr = reinterpret_cast<const ev4sio_physx::PxU32*>(&mask);
	atomicOr(address32, maskPtr[0]);
	atomicOr(address32 + 1, maskPtr[1]);
}

/* use inline assembly with .global qualifier to perform the operation at the L2 cache
 * adds 20% performance in FLIP P2G compared to atomicAdd() or plain red.add.f32 */
PX_FORCE_INLINE __device__ void PxRedAddGlobal(float* addr, const float val)
{
#if __CUDA_ARCH__ >= 350
	asm volatile ("red.global.add.f32 [%0], %1;" :: __STG_PTR(addr) , "f"(val));
#else
#if __CUDA_ARCH__ >= 200
	atomicAdd(addr, val);
#else
	PX_UNUSED(addr);
	PX_UNUSED(val);
#endif
#endif
}

#endif