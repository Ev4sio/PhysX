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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PHYSX_SNIPPET_KINEMATIC_SOFTBODY_H
#define PHYSX_SNIPPET_KINEMATIC_SOFTBODY_H

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include <vector>

class SoftBody
{
public:
	SoftBody(ev4sio_physx::PxSoftBody* softBody, ev4sio_physx::PxCudaContextManager* cudaContextManager) :
		mSoftBody(softBody),
		mCudaContextManager(cudaContextManager)
	{
		mPositionsInvMass = PX_PINNED_HOST_ALLOC_T(ev4sio_physx::PxVec4, cudaContextManager, softBody->getCollisionMesh()->getNbVertices());
	}

	~SoftBody()
	{
	}

	void release()
	{
		if (mSoftBody)
			mSoftBody->release();
		if (mPositionsInvMass)
			PX_PINNED_HOST_FREE(mCudaContextManager, mPositionsInvMass);

		if (mTargetPositionsH)
			PX_PINNED_HOST_FREE(mCudaContextManager, mTargetPositionsH);
		if (mTargetPositionsD)
			PX_DEVICE_FREE(mCudaContextManager, mTargetPositionsD);
	}

	void copyDeformedVerticesFromGPUAsync(CUstream stream)
	{	
		ev4sio_physx::PxTetrahedronMesh* tetMesh = mSoftBody->getCollisionMesh();

		ev4sio_physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoHAsync(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mSoftBody->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(ev4sio_physx::PxVec4), stream);
	}

	void copyDeformedVerticesFromGPU()
	{	
		ev4sio_physx::PxTetrahedronMesh* tetMesh = mSoftBody->getCollisionMesh();

		ev4sio_physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoH(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mSoftBody->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(ev4sio_physx::PxVec4));
	}


	ev4sio_physx::PxVec4* mPositionsInvMass;
	ev4sio_physx::PxSoftBody* mSoftBody;
	ev4sio_physx::PxCudaContextManager* mCudaContextManager;

	ev4sio_physx::PxVec4* mTargetPositionsH;
	ev4sio_physx::PxVec4* mTargetPositionsD;
	ev4sio_physx::PxU32 mTargetCount;
};

#endif
