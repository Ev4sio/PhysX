// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_ISLAND_MANAGER_H
#define DY_ISLAND_MANAGER_H

// PT: low-level-dynamics client helper code for using the low-level island sim

#include "PxsIslandSim.h"

namespace ev4sio_physx
{
	class PxsRigidBody;

	namespace ev4sio_Dy
	{
		class FeatherstoneArticulation;
#if PX_SUPPORT_GPU_PHYSX
		class DeformableSurface;
		class DeformableVolume;
		class ParticleSystem;
#endif
	}

	template <typename T>
	struct IGNodeTraits
	{
		enum {TypeID = ev4sio_IG::Node::eTYPE_COUNT };
	};
	template <typename T> struct IGNodeTraits<const T> { enum { TypeID = IGNodeTraits<T>::TypeID }; };

	template <> struct IGNodeTraits<PxsRigidBody>					{ enum { TypeID = ev4sio_IG::Node::eRIGID_BODY_TYPE };			};
	template <> struct IGNodeTraits<ev4sio_Dy::FeatherstoneArticulation>	{ enum { TypeID = ev4sio_IG::Node::eARTICULATION_TYPE };		};

#if PX_SUPPORT_GPU_PHYSX
	template <> struct IGNodeTraits<ev4sio_Dy::DeformableSurface>			{ enum { TypeID = ev4sio_IG::Node::eDEFORMABLE_SURFACE_TYPE };	};
	template <> struct IGNodeTraits<ev4sio_Dy::DeformableVolume>			{ enum { TypeID = ev4sio_IG::Node::eDEFORMABLE_VOLUME_TYPE };	};
	template <> struct IGNodeTraits<ev4sio_Dy::ParticleSystem>				{ enum { TypeID = ev4sio_IG::Node::ePARTICLESYSTEM_TYPE };		};
#endif

	template<class T>
	PX_FORCE_INLINE T* getObjectFromIG(const ev4sio_IG::Node& node)
	{
		PX_ASSERT(PxU32(node.mType) == PxU32(IGNodeTraits<T>::TypeID));
		return reinterpret_cast<T*>(node.mObject);
	}

	PX_FORCE_INLINE PxsRigidBody* getRigidBodyFromIG(const ev4sio_IG::IslandSim& islandSim, PxNodeIndex nodeIndex)
	{
		return reinterpret_cast<PxsRigidBody*>(islandSim.getObject(nodeIndex, ev4sio_IG::Node::eRIGID_BODY_TYPE));
	}

	PX_FORCE_INLINE ev4sio_Dy::FeatherstoneArticulation* getArticulationFromIG(const ev4sio_IG::IslandSim& islandSim, PxNodeIndex nodeIndex)
	{
		return reinterpret_cast<ev4sio_Dy::FeatherstoneArticulation*>(islandSim.getObject(nodeIndex, ev4sio_IG::Node::eARTICULATION_TYPE));
	}
}

#endif