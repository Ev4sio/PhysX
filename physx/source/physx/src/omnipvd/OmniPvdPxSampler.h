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

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

#if PX_SUPPORT_OMNI_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxMutex.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxErrorCallback.h"
#include "OmniPvdChunkAlloc.h"

namespace ev4sio_physx
{
	class PxScene;
	class PxBase;
	class NpScene;
	class PxActor;
	class PxShape;
	class PxMaterial;

	class PxArticulationReducedCoordinate;
	class PxArticulationJointReducedCoordinate;
	class PxArticulationLink;
	class PxRigidDynamic;

	class PxDeformableMaterial;
	class PxDeformableSurfaceMaterial;
	class PxDeformableVolumeMaterial;
	class PxPBDMaterial;
	class PxDiffuseParticleParams;

	struct OmniPvdPxCoreRegistrationData;

	class NpOmniPvd;
}

void streamActorName(const ev4sio_physx::PxActor & a, const char* name);
void streamSceneName(const ev4sio_physx::PxScene & s, const char* name);
void streamArticulationName(const ev4sio_physx::PxArticulationReducedCoordinate & art, const char* name);
void streamArticulationJointName(const ev4sio_physx::PxArticulationJointReducedCoordinate& joint, const char* name);

void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);

void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxDeformableMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);
void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxDeformableSurfaceMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);
void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxDeformableVolumeMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);
void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxPBDMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);

void streamDiffuseParticleParamsAttributes(const ev4sio_physx::PxDiffuseParticleParams& diffuseParams);

enum OmniPvdSharedMeshEnum {
	eOmniPvdTriMesh     = 0,
	eOmniPvdConvexMesh  = 1,
	eOmniPvdHeightField = 2,
};

class OmniPvdWriter;

namespace ev4sio_physx
{

class NpOmniPvdSceneClient : public ev4sio_physx::PxUserAllocated
{
public:
	NpOmniPvdSceneClient(ev4sio_physx::PxScene& scene);
	~NpOmniPvdSceneClient();	

	////////////////////////////////////////////////////////////////////////////////
	// Regarding the frame sampling strategy, the OVD frames start at (1:odd) with the first
	// one being a pre-Sim frame, for the setup calls done on the NpScene, in the constructor
	// as well as any user set operations once the scene was created, but not yet simulated.
	// 
	// After the first simulate call, the second frame (2:even), considers all the data recorded
	// up until the end of fetchresults as post-Sim.
	// 
	// Once fetchresults has exited, all the subsequent data is considered as pre-Sim data (odd frames)
	// 
	// Similarly for any subsequent simulate call, the data is considered post-Sim (evem frames)
	// 
	// A diagram of how this is layed out
	// 
	//  NpScene::NpScene()
	//    [pre-Sim data]  : frame 1   (odd frame)
	//  NpScene::simulate()
	//    [post-Sim data] : frame 2   (even frame)
	//  NpScene::fetchresults()
	//    [pre-Sim data]  : frame n+1 (odd frame)
	//  NpScene::simulate()
	//    [post-Sim data] : frame n+2 (even frame)
	//  NpScene::fetchresults()
	// 
	////////////////////////////////////////////////////////////////////////////////

	void startFirstFrame(OmniPvdWriter& pvdWriter);
	void incrementFrame(OmniPvdWriter& pvdWriter, bool recordProfileFrame = false); // stopFrame (frameID), then startFrame (frameID + 1)
	void stopLastFrame(OmniPvdWriter& pvdWriter);
	
	void addRigidDynamicReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicForceReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicTorqueReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	void removeRigidDynamicReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	
	void addArticulationFromLinkFlagChangeReset(const ev4sio_physx::PxArticulationLink* link);
	void addArticulationLinksForceReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationLinksTorqueReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationJointsForceReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	void removeArticulationReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	
	void resetForces();

private:
	ev4sio_physx::PxScene& mScene;
	ev4sio_physx::PxU64 mFrameId;

	ev4sio_physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicForce;
	ev4sio_physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicTorque;

	ev4sio_physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksForce;
	ev4sio_physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksTorque;
	ev4sio_physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationJointsForce;
};

}

class OmniPvdPxSampler : public ev4sio_physx::PxUserAllocated, public ev4sio_physx::PxErrorCallback
{
public:
	OmniPvdPxSampler();
	~OmniPvdPxSampler();
	bool startSampling();
	bool isSampling();
	void setOmniPvdInstance(ev4sio_physx::NpOmniPvd* omniPvdIntance);

	// writes all contacts to the stream
	void streamSceneContacts(ev4sio_physx::NpScene& scene);

	static OmniPvdPxSampler* getInstance();

	void onObjectAdd(const ev4sio_physx::PxBase& object);
	void onObjectRemove(const ev4sio_physx::PxBase& object);
	
	virtual void reportError(ev4sio_physx::PxErrorCode::Enum code, const char* message, const char* file, int line) PX_OVERRIDE;
};


namespace ev4sio_physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData();
NpOmniPvd* NpOmniPvdGetInstance();

}

#endif

#endif
