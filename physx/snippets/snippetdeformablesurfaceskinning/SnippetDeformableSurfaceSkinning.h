// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PHYSX_SNIPPET_DEFORMABLE_SURFACE_SKINNING_H
#define PHYSX_SNIPPET_DEFORMABLE_SURFACE_SKINNING_H

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "PxDeformableSurface.h"
#include "geometry/PxTriangle.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "extensions/PxCudaHelpersExt.h"
#include "foundation/PxVec3.h"
#include "foundation/PxArray.h"


struct SkinnedMesh
{
	ev4sio_physx::PxArray<ev4sio_physx::PxVec3> mVertices;
	ev4sio_physx::PxArray<ev4sio_physx::PxU32> mTriangles;
};

struct BasePostSolveCallback : ev4sio_physx::PxPostSolveCallback
{
	virtual void synchronize() = 0;
	virtual ev4sio_physx::PxVec3* getSkinnedVertices(ev4sio_physx::PxU32 clothIndex) = 0;
};

class DeformableSurface
{
public:
	DeformableSurface() :
		mPositionsInvMass(NULL),
		mDeformableSurface(NULL),
		mCudaContextManager(NULL),
		mTriangleMesh(NULL)
	{ }

	DeformableSurface(ev4sio_physx::PxDeformableSurface* deformableSurface, ev4sio_physx::PxCudaContextManager* cudaContextManager) :
		mDeformableSurface(deformableSurface),
		mCudaContextManager(cudaContextManager)
	{
		ev4sio_physx::PxShape* shape = deformableSurface->getShape();

		const ev4sio_physx::PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const ev4sio_physx::PxTriangleMeshGeometry&>(shape->getGeometry());
		mTriangleMesh = triangleMeshGeom.triangleMesh;

		mPositionsInvMass = PX_EXT_PINNED_MEMORY_ALLOC(ev4sio_physx::PxVec4, *cudaContextManager, mTriangleMesh->getNbVertices());
	}

	~DeformableSurface()
	{
	}

	void release()
	{
		if (mDeformableSurface)
			mDeformableSurface->release();
		PX_EXT_PINNED_MEMORY_FREE(*mCudaContextManager, mPositionsInvMass);
	}

	void copyDeformedVerticesFromGPUAsync(CUstream stream)
	{	
		ev4sio_physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoHAsync(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableSurface->getPositionInvMassBufferD()), mTriangleMesh->getNbVertices() * sizeof(ev4sio_physx::PxVec4), stream);
	}

	void copyDeformedVerticesFromGPU()
	{	
		ev4sio_physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoH(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableSurface->getPositionInvMassBufferD()), mTriangleMesh->getNbVertices() * sizeof(ev4sio_physx::PxVec4));
	}


	ev4sio_physx::PxVec4* mPositionsInvMass;
	ev4sio_physx::PxDeformableSurface* mDeformableSurface;
	ev4sio_physx::PxCudaContextManager* mCudaContextManager;
	ev4sio_physx::PxTriangleMesh* mTriangleMesh;
};

#endif
