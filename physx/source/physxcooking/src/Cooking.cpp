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

#include "Cooking.h"
#include "GuCooking.h"
#include "GuBVH.h"

///////////////////////////////////////////////////////////////////////////////

using namespace ev4sio_physx;
using namespace ev4sio_Gu;

#include "cooking/PxCookingInternal.h"
#include "GuTriangleMeshBV4.h"
ev4sio_physx::PxTriangleMesh* PxCreateTriangleMeshInternal(const ev4sio_physx::PxTriangleMeshInternalData& data)
{
	TriangleMesh* np;
	PX_NEW_SERIALIZED(np, BV4TriangleMesh)(data);
	return np;
}

ev4sio_physx::PxBVH* PxCreateBVHInternal(const ev4sio_physx::PxBVHInternalData& data)
{
	BVH* np;
	PX_NEW_SERIALIZED(np, BVH)(data);
	return np;
}

///////////////////////////////////////////////////////////////////////////////

PxInsertionCallback* ev4sio_PxGetStandaloneInsertionCallback()
{
	return immediateCooking::getInsertionCallback();
}

bool ev4sio_PxCookBVH(const PxBVHDesc& desc, PxOutputStream& stream)
{
	return immediateCooking::cookBVH(desc, stream);
}

PxBVH* ev4sio_PxCreateBVH(const PxBVHDesc& desc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createBVH(desc, insertionCallback);
}

bool ev4sio_PxCookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream)
{
	return immediateCooking::cookHeightField(desc, stream);
}

PxHeightField* ev4sio_PxCreateHeightField(const PxHeightFieldDesc& desc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createHeightField(desc, insertionCallback);
}

bool ev4sio_PxCookConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition)
{
	return immediateCooking::cookConvexMesh(params, desc, stream, condition);
}

PxConvexMesh* ev4sio_PxCreateConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc, PxInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition)
{
	return immediateCooking::createConvexMesh(params, desc, insertionCallback, condition);
}

bool ev4sio_PxValidateConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc)
{
	return immediateCooking::validateConvexMesh(params, desc);
}

bool ev4sio_PxComputeHullPolygons(const PxCookingParams& params, const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback, PxU32& nbVerts, PxVec3*& vertices, PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons)
{
	return immediateCooking::computeHullPolygons(params, mesh, inCallback, nbVerts, vertices, nbIndices, indices, nbPolygons, hullPolygons);
}

bool ev4sio_PxValidateTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc)
{
	return immediateCooking::validateTriangleMesh(params, desc);
}

PxTriangleMesh* ev4sio_PxCreateTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition)
{
	return immediateCooking::createTriangleMesh(params, desc, insertionCallback, condition);
}

bool ev4sio_PxCookTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition)
{
	return immediateCooking::cookTriangleMesh(params, desc, stream, condition);
}

bool ev4sio_PxCookTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxOutputStream& stream)
{
	return immediateCooking::cookTetrahedronMesh(params, meshDesc, stream);
}

PxTetrahedronMesh* ev4sio_PxCreateTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createTetrahedronMesh(params, meshDesc, insertionCallback);
}

bool ev4sio_PxCookDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
	const PxDeformableVolumeSimulationDataDesc& softbodyDataDesc, PxOutputStream& stream)
{
	return immediateCooking::cookDeformableVolumeMesh(params, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, stream);
}

PxDeformableVolumeMesh* ev4sio_PxCreateDeformableVolumeMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc,
	const PxTetrahedronMeshDesc& collisionMeshDesc, const PxDeformableVolumeSimulationDataDesc& softbodyDataDesc, PxInsertionCallback& insertionCallback)
{
	return immediateCooking::createDeformableVolumeMesh(params, simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, insertionCallback);
}

PxCollisionMeshMappingData* ev4sio_PxComputeModelsMapping(const PxCookingParams& params, PxTetrahedronMeshData& simulationMesh,
	const PxTetrahedronMeshData& collisionMesh, const PxDeformableVolumeCollisionData& collisionData, const PxBoundedData* vertexToTet)
{
	return immediateCooking::computeModelsMapping(params, simulationMesh, collisionMesh, collisionData, vertexToTet);
}
	
PxCollisionTetrahedronMeshData* ev4sio_PxComputeCollisionData(const PxCookingParams& params, const PxTetrahedronMeshDesc& collisionMeshDesc)
{
	return immediateCooking::computeCollisionData(params, collisionMeshDesc);
}

PxSimulationTetrahedronMeshData* ev4sio_PxComputeSimulationData(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc)
{
	return immediateCooking::computeSimulationData(params, simulationMeshDesc);
}

PxDeformableVolumeMesh* ev4sio_PxAssembleDeformableVolumeMesh(PxTetrahedronMeshData& simulationMesh, PxDeformableVolumeSimulationData& simulationData,
	PxTetrahedronMeshData& collisionMesh, PxDeformableVolumeCollisionData& collisionData, PxCollisionMeshMappingData& mappingData,
	PxInsertionCallback& insertionCallback)
{
	return immediateCooking::assembleDeformableVolumeMesh(simulationMesh, simulationData, collisionMesh, collisionData, mappingData, insertionCallback);
}

