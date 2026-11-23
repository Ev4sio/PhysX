// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_TRIANGLE_MESH_H
#define GU_COOKING_TRIANGLE_MESH_H

#include "GuMeshData.h"
#include "cooking/PxCooking.h"

namespace ev4sio_physx
{
	namespace ev4sio_Gu
	{
		class EdgeList;
	}

	class TriangleMeshBuilder
	{
		public:
											TriangleMeshBuilder(ev4sio_Gu::TriangleMeshData& mesh, const PxCookingParams& params);
		virtual								~TriangleMeshBuilder();

		virtual	PxMeshMidPhase::Enum		getMidphaseID()									const	= 0;
		// Called by base code when midphase structure should be built
		virtual	bool						createMidPhaseStructure()								= 0;

		// Called by base code when midphase structure should be saved
		virtual	void						saveMidPhaseStructure(PxOutputStream& stream, bool mismatch)	const	= 0;
		// Called by base code when mesh index format has changed and the change should be reflected in midphase structure
		virtual	void						onMeshIndexFormatChange()								{}

				bool						cleanMesh(bool validate, PxTriangleMeshCookingResult::Enum* condition);
				void						remapTopology(const PxU32* order);
		
				void						createVertMapping();
				void						createSharedEdgeData(bool buildAdjacencies, bool buildActiveEdges);

				void						recordTriangleIndices();
				bool						createGRBMidPhaseAndData(const PxU32 originalTriangleCount);
				void						createGRBData();

				bool						loadFromDesc(const PxTriangleMeshDesc&, PxTriangleMeshCookingResult::Enum* condition, bool validate = false);
				
				bool						save(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params) const;
				void						checkMeshIndicesSize();
	PX_FORCE_INLINE	ev4sio_Gu::TriangleMeshData&	getMeshData()	{ return mMeshData;	}
	protected:
				bool						importMesh(const PxTriangleMeshDesc& desc, PxTriangleMeshCookingResult::Enum* condition, bool validate = false);

				bool						loadFromDescInternal(PxTriangleMeshDesc&, PxTriangleMeshCookingResult::Enum* condition, bool validate = false);

				void						buildInertiaTensor(bool flipNormals = false);
				void						buildInertiaTensorFromSDF();
				void						normalizeSDFCollisionMeshWinding(bool flipNormalsRequested);

				TriangleMeshBuilder& operator=(const TriangleMeshBuilder&);
				ev4sio_Gu::EdgeList*				mEdgeList;
				const PxCookingParams&		mParams;
				ev4sio_Gu::TriangleMeshData&		mMeshData;
	};

	class RTreeTriangleMeshBuilder : public TriangleMeshBuilder
	{
		public:
											RTreeTriangleMeshBuilder(const PxCookingParams& params);
		virtual								~RTreeTriangleMeshBuilder();

		virtual	PxMeshMidPhase::Enum		getMidphaseID()	const		PX_OVERRIDE	{ return PxMeshMidPhase::eBVH33;	}
		virtual	bool						createMidPhaseStructure()	PX_OVERRIDE;
		virtual	void						saveMidPhaseStructure(PxOutputStream& stream, bool mismatch)	const	PX_OVERRIDE;

				ev4sio_Gu::RTreeTriangleData		mData;
	};

	class BV4TriangleMeshBuilder : public TriangleMeshBuilder
	{
		public:
											BV4TriangleMeshBuilder(const PxCookingParams& params);
		virtual								~BV4TriangleMeshBuilder();

		virtual	PxMeshMidPhase::Enum		getMidphaseID()	const		PX_OVERRIDE	{ return PxMeshMidPhase::eBVH34;	}
		virtual	bool						createMidPhaseStructure()	PX_OVERRIDE;
		virtual	void						saveMidPhaseStructure(PxOutputStream& stream, bool mismatch)	const	PX_OVERRIDE;
		virtual	void						onMeshIndexFormatChange() PX_OVERRIDE;

				ev4sio_Gu::BV4TriangleData			mData;
	};

	class BV32TriangleMeshBuilder
	{
	public:
		static	bool						createMidPhaseStructure(const PxCookingParams& params, ev4sio_Gu::TriangleMeshData& meshData, ev4sio_Gu::BV32Tree& bv32Tree);
		static	void						saveMidPhaseStructure(ev4sio_Gu::BV32Tree* tree, PxOutputStream& stream, bool mismatch);
	};

}

#endif
