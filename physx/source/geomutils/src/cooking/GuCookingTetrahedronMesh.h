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

#ifndef GU_COOKING_TETRAHEDRON_MESH_H
#define GU_COOKING_TETRAHEDRON_MESH_H

#include "cooking/PxCooking.h"
#include "GuMeshData.h"

namespace ev4sio_physx
{
	class TetrahedronMeshBuilder
	{
		PX_NOCOPY(TetrahedronMeshBuilder)
	public:

		static bool	loadFromDesc(const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
			PxDeformableVolumeSimulationDataDesc deformableVolumeDataDesc, ev4sio_Gu::TetrahedronMeshData& simulationMesh, ev4sio_Gu::DeformableVolumeSimulationData& simulationData,
			ev4sio_Gu::TetrahedronMeshData& collisionMesh, ev4sio_Gu::DeformableVolumeCollisionData& collisionData, ev4sio_Gu::CollisionMeshMappingData& mappingData,
			const PxCookingParams&	params, bool validateMesh = false);

		static bool	saveTetrahedronMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params,
			const ev4sio_Gu::TetrahedronMeshData& mesh);

		static bool	saveDeformableVolumeMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params,
			const ev4sio_Gu::TetrahedronMeshData& simulationMesh, const ev4sio_Gu::DeformableVolumeSimulationData& simulationData,
			const ev4sio_Gu::TetrahedronMeshData& collisionMesh, const ev4sio_Gu::DeformableVolumeCollisionData& collisionData,
			const ev4sio_Gu::CollisionMeshMappingData& mappingData);

		//PxMeshMidPhase::Enum				getMidphaseID()	const { return PxMeshMidPhase::eBVH34; }
		static bool	createMidPhaseStructure(ev4sio_Gu::TetrahedronMeshData& collisionMesh, ev4sio_Gu::DeformableVolumeCollisionData& collisionData, const PxCookingParams& params);
		static void	saveMidPhaseStructure(PxOutputStream& stream, bool mismatch, const ev4sio_Gu::DeformableVolumeCollisionData& collisionData);

		static void	computeTetData(const PxTetrahedronMeshDesc& desc, ev4sio_Gu::TetrahedronMeshData& mesh);

		static bool	createGRBMidPhaseAndData(const PxU32 originalTriangleCount, ev4sio_Gu::TetrahedronMeshData& collisionMesh, ev4sio_Gu::DeformableVolumeCollisionData& collisionData, const PxCookingParams& params);
		static void	computeSimData(const PxTetrahedronMeshDesc& desc, ev4sio_Gu::TetrahedronMeshData& simulationMesh, ev4sio_Gu::DeformableVolumeSimulationData& simulationData, const PxCookingParams& params);
		static void	computeModelsMapping(ev4sio_Gu::TetrahedronMeshData& simulationMesh, const ev4sio_Gu::TetrahedronMeshData& collisionMesh, const ev4sio_Gu::DeformableVolumeCollisionData& collisionData,
																	ev4sio_Gu::CollisionMeshMappingData& mappingData, bool buildGPUData, const PxBoundedData* vertexToTet);
		static void	createCollisionModelMapping(const ev4sio_Gu::TetrahedronMeshData& collisionMesh, const ev4sio_Gu::DeformableVolumeCollisionData& collisionData, ev4sio_Gu::CollisionMeshMappingData& mappingData);
		
		static void	recordTetrahedronIndices(const ev4sio_Gu::TetrahedronMeshData& collisionMesh, ev4sio_Gu::DeformableVolumeCollisionData& collisionData, bool buildGPUData);
		static bool	importMesh(const PxTetrahedronMeshDesc& collisionMeshDesc, const PxCookingParams& params, 
								ev4sio_Gu::TetrahedronMeshData& collisionMesh, ev4sio_Gu::DeformableVolumeCollisionData& collisionData, bool validate = false);
		
		static bool	computeCollisionData(const PxTetrahedronMeshDesc& collisionMeshDesc, ev4sio_Gu::TetrahedronMeshData& collisionMesh, ev4sio_Gu::DeformableVolumeCollisionData& collisionData,
										const PxCookingParams&	params, bool validateMesh = false);
	};

	class BV32TetrahedronMeshBuilder
	{
	public:
		static	bool	createMidPhaseStructure(const PxCookingParams& params, ev4sio_Gu::TetrahedronMeshData& meshData, ev4sio_Gu::BV32Tree& bv32Tree, ev4sio_Gu::DeformableVolumeCollisionData& collisionData);
		static	void	saveMidPhaseStructure(ev4sio_Gu::BV32Tree* tree, PxOutputStream& stream, bool mismatch);
	};
}

#endif
