// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_MESH_FACTORY_H
#define GU_MESH_FACTORY_H

#include "foundation/PxIO.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxUserAllocated.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxHeightField.h"
#include "geometry/PxBVH.h"
#include "PxPhysXConfig.h"

#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"

// PT: added for platforms that compile the onRefCountZero template immediately
#include "CmUtils.h"
#include "foundation/PxFoundation.h"

namespace ev4sio_physx
{
namespace ev4sio_Gu
{
	class ConvexMesh;
	class HeightField;
	class TriangleMesh;
	class TriangleMeshData;
	class DeformableVolumeMesh;
	class DeformableVolumeMeshData;
	class TetrahedronMesh;
	class TetrahedronMeshData;
	class BVH;
	struct ConvexHullInitData;
	class BVHData;

	class MeshFactoryListener
	{
	protected:
		virtual ~MeshFactoryListener(){}
	public:
		virtual void onMeshFactoryBufferRelease(const PxBase* object, PxType type) = 0;
#if PX_SUPPORT_OMNI_PVD
		virtual void onObjectAdd(const PxBase*) {}
		virtual void onObjectRemove(const PxBase*) {}
#endif
	};

	#if PX_VC 
		#pragma warning(push)
		#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
	#endif

	class PX_PHYSX_COMMON_API MeshFactory : public PxUserAllocated
	{
		PX_NOCOPY(MeshFactory)
	public:
										MeshFactory();
	protected:
		virtual							~MeshFactory();

	public:
		void							release();

		// Triangle meshes
		void							addTriangleMesh(ev4sio_Gu::TriangleMesh* np, bool lock=true);
		PxTriangleMesh*					createTriangleMesh(PxInputStream& stream);
		PxTriangleMesh*					createTriangleMesh(void* triangleMeshData);
		bool							removeTriangleMesh(PxTriangleMesh&);
		PxU32							getNbTriangleMeshes()	const;
		PxU32							getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Tetrahedron meshes
		void							addTetrahedronMesh(ev4sio_Gu::TetrahedronMesh* np, bool lock = true);
		PxTetrahedronMesh*				createTetrahedronMesh(PxInputStream& stream);
		PxTetrahedronMesh*				createTetrahedronMesh(void* tetrahedronMeshData);
		bool							removeTetrahedronMesh(PxTetrahedronMesh&);
		PxU32							getNbTetrahedronMeshes()	const;
		PxU32							getTetrahedronMeshes(PxTetrahedronMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Deformable volume meshes
		void							addDeformableVolumeMesh(ev4sio_Gu::DeformableVolumeMesh* np, bool lock = true);
		PxDeformableVolumeMesh*			createDeformableVolumeMesh(PxInputStream& stream);
		PxDeformableVolumeMesh*			createDeformableVolumeMesh(void* tetrahedronMeshData);
		bool							removeDeformableVolumeMesh(PxDeformableVolumeMesh&);
		PxU32							getNbDeformableVolumeMeshes()	const;
		PxU32							getDeformableVolumeMeshes(PxDeformableVolumeMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Convexes
		void							addConvexMesh(ev4sio_Gu::ConvexMesh* np, bool lock=true);
		PxConvexMesh*					createConvexMesh(PxInputStream&);
		PxConvexMesh*					createConvexMesh(void* convexMeshData);
		bool							removeConvexMesh(PxConvexMesh&);
		PxU32							getNbConvexMeshes() const;
		PxU32							getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Heightfields
		void							addHeightField(ev4sio_Gu::HeightField* np, bool lock=true);
		PxHeightField*					createHeightField(void* heightFieldMeshData);
		PxHeightField*					createHeightField(PxInputStream&);
		bool							removeHeightField(PxHeightField&);
		PxU32							getNbHeightFields()	const;
		PxU32							getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// BVH
		void							addBVH(ev4sio_Gu::BVH* np, bool lock=true);
		PxBVH*							createBVH(PxInputStream&);
		PxBVH*							createBVH(void* bvhData);
		bool							removeBVH(PxBVH&);
		PxU32							getNbBVHs() const;
		PxU32							getBVHs(PxBVH** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		void							addFactoryListener(MeshFactoryListener& listener);
		void							removeFactoryListener(MeshFactoryListener& listener);
		void							notifyFactoryListener(const PxBase*, PxType typeID);

		bool							remove(PxBase&);

	protected:

		PxTriangleMesh*					createTriangleMesh(ev4sio_Gu::TriangleMeshData& data);
		PxTetrahedronMesh*				createTetrahedronMesh(ev4sio_Gu::TetrahedronMeshData& data);
		PxDeformableVolumeMesh*			createDeformableVolumeMesh(ev4sio_Gu::DeformableVolumeMeshData& data);
		PxConvexMesh*					createConvexMesh(ev4sio_Gu::ConvexHullInitData& data);
		PxBVH*							createBVH(ev4sio_Gu::BVHData& data);

		mutable PxMutex					mTrackingMutex;
	private:
		PxCoalescedHashSet<ev4sio_Gu::TriangleMesh*>			mTriangleMeshes;
		PxCoalescedHashSet<ev4sio_Gu::TetrahedronMesh*>		mTetrahedronMeshes;
		PxCoalescedHashSet<ev4sio_Gu::DeformableVolumeMesh*>	mDeformableVolumeMeshes;
		PxCoalescedHashSet<ev4sio_Gu::ConvexMesh*>				mConvexMeshes;
		PxCoalescedHashSet<ev4sio_Gu::HeightField*>			mHeightFields;
		PxCoalescedHashSet<ev4sio_Gu::BVH*>					mBVHs;

		PxArray<MeshFactoryListener*>					mFactoryListeners;

#if PX_SUPPORT_OMNI_PVD
	protected:
		void							notifyListenersAdd(const PxBase*);
		void							notifyListenersRemove(const PxBase*);
#endif
	};
	#if PX_VC 
		 #pragma warning(pop) 
	#endif

		template<typename T>
		PX_INLINE void onRefCountZero(T* object, ev4sio_Gu::MeshFactory* mf, bool cndt, const char* errorMsg)
		{
			if(mf)
			{
				if(cndt || mf->remove(*object))
				{
					const PxType type = object->getConcreteType();
					ev4sio_Cm::deletePxBase(object);
					mf->notifyFactoryListener(object, type);
					return;
				}
	
				// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
				// This prevents deleting the object twice.
				ev4sio_PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, errorMsg);
			}
			else
				ev4sio_Cm::deletePxBase(object);
		}
	}

}
#endif
