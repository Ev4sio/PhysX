// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PHYSX_SNIPPET_DEFORMABLE_VOLUME_SKINNING_H
#define PHYSX_SNIPPET_DEFORMABLE_VOLUME_SKINNING_H

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "extensions/PxCudaHelpersExt.h"

struct SkinnedMesh
{
	ev4sio_physx::PxArray<ev4sio_physx::PxVec3> mVertices;
	ev4sio_physx::PxArray<ev4sio_physx::PxU32> mTriangles;
};

struct BasePostSolveCallback : ev4sio_physx::PxPostSolveCallback
{
	virtual void synchronize() = 0;
	virtual ev4sio_physx::PxVec3* getSkinnedVertices(ev4sio_physx::PxU32 deformableVolumeIndex) = 0;
};

class DeformableVolume
{
public:
	DeformableVolume() :
		mPositionsInvMass(NULL),
		mDeformableVolume(NULL),
		mCudaContextManager(NULL)
	{ }

	DeformableVolume(ev4sio_physx::PxDeformableVolume* deformableVolume, ev4sio_physx::PxCudaContextManager* cudaContextManager) :
		mDeformableVolume(deformableVolume),
		mCudaContextManager(cudaContextManager)
	{
		mPositionsInvMass = PX_EXT_PINNED_MEMORY_ALLOC(ev4sio_physx::PxVec4, *cudaContextManager, deformableVolume->getCollisionMesh()->getNbVertices());
	}

	~DeformableVolume()
	{
	}

	void release()
	{
		if (mDeformableVolume)
			mDeformableVolume->release();

		PX_EXT_PINNED_MEMORY_FREE(*mCudaContextManager, mPositionsInvMass);
	}

	void copyDeformedVerticesFromGPUAsync(CUstream stream)
	{
		ev4sio_physx::PxTetrahedronMesh* tetMesh = mDeformableVolume->getCollisionMesh();

		ev4sio_physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoHAsync(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableVolume->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(ev4sio_physx::PxVec4), stream);
	}

	void copyDeformedVerticesFromGPU()
	{
		ev4sio_physx::PxTetrahedronMesh* tetMesh = mDeformableVolume->getCollisionMesh();

		ev4sio_physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoH(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableVolume->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(ev4sio_physx::PxVec4));
	}


	ev4sio_physx::PxVec4* mPositionsInvMass;
	ev4sio_physx::PxDeformableVolume* mDeformableVolume;
	ev4sio_physx::PxCudaContextManager* mCudaContextManager;
};

#endif
