// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_PX_SAMPLER_H
#define OMNI_PVD_PX_SAMPLER_H

#if PX_SUPPORT_OMNI_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxMutex.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxErrorCallback.h"
#include "common/PxPhysXCommonConfig.h"
#include "OmniPvdChunkAlloc.h"
#include "omnipvd/PxOmniPvd.h"

namespace ev4sio_physx
{
	class PxScene;
	class PxBase;
	class NpScene;
	class PxActor;
	class PxShape;
	class PxConstraint;
	class PxConstraintConnector;
	class PxMaterial;

	class PxArticulationReducedCoordinate;
	class PxArticulationJointReducedCoordinate;
	class PxArticulationLink;
	class PxRigidDynamic;
	class PxRigidActor;
	class PxRigidBody;
	class PxArticulationMimicJoint;

	namespace ev4sio_Sc { class BodyCore; }

	class PxDeformableBody;
	class PxDeformableSurface;
	class PxDeformableSurfaceMaterial;
	class PxDeformableVolume;
	class PxDeformableVolumeMaterial;
	class PxPBDMaterial;
	class PxDiffuseParticleParams;
	class PxParticleBuffer;
	class PxPBDParticleSystem;

	struct OmniPvdPxCoreRegistrationData;

	class NpOmniPvd;
}

class OmniPvdWriter;

void streamActorName(const ev4sio_physx::PxActor & a, const char* name);
void streamSceneName(const ev4sio_physx::PxScene & s, const char* name);
void streamArticulationName(const ev4sio_physx::PxArticulationReducedCoordinate & art, const char* name);
void streamArticulationJointName(const ev4sio_physx::PxArticulationJointReducedCoordinate& joint, const char* name);
void streamParticleBufferName(const ev4sio_physx::PxParticleBuffer& pb, const char* name);
// Explicit forms used by the full-state snapshot (caller already holds a write scope).
void streamSceneName(OmniPvdWriter* pvdWriter, const ev4sio_physx::OmniPvdPxCoreRegistrationData* pvdRegData, const ev4sio_physx::PxScene& s, const char* name);
void streamArticulationName(OmniPvdWriter* pvdWriter, const ev4sio_physx::OmniPvdPxCoreRegistrationData* pvdRegData, const ev4sio_physx::PxArticulationReducedCoordinate& art, const char* name);
void streamArticulationJointName(OmniPvdWriter* pvdWriter, const ev4sio_physx::OmniPvdPxCoreRegistrationData* pvdRegData, const ev4sio_physx::PxArticulationJointReducedCoordinate& joint, const char* name);

void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);

void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxDeformableSurfaceMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);
void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxDeformableVolumeMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);
void streamShapeMaterials(const ev4sio_physx::PxShape&, ev4sio_physx::PxPBDMaterial* const * mats, ev4sio_physx::PxU32 nbrMaterials);

void streamDiffuseParticleParamsAttributes(const ev4sio_physx::PxDiffuseParticleParams& diffuseParams);

void streamArticulationMimicJoint(const ev4sio_physx::PxArticulationMimicJoint& mj);

void streamShapeUpdateGeometry(const ev4sio_physx::PxShape& shape);

void streamDeformableVolumeAttributes(const ev4sio_physx::PxDeformableVolume& dv);
void streamDeformableSurfaceAttributes(const ev4sio_physx::PxDeformableSurface& ds);

enum OmniPvdSharedMeshEnum {
	eOmniPvdTriMesh     = 0,
	eOmniPvdConvexMesh  = 1,
	eOmniPvdHeightField = 2,
	eOmniPvdTetraMesh   = 3,
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
	void resetFrameId(); // rewind mFrameId to 1 (odd = pre-sim) for a snapshot onto a fresh stream
	
	void addRigidDynamicReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicForceReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	void addRigidDynamicTorqueReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	void removeRigidDynamicReset(const ev4sio_physx::PxRigidDynamic* rigidDynamic);
	
	void addArticulationFromLinkFlagChangeReset(const ev4sio_physx::PxArticulationLink* link);
	void addArticulationLinksForceReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	void addArticulationLinksTorqueReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	void removeArticulationReset(const ev4sio_physx::PxArticulationReducedCoordinate* articulation);
	
	void resetForces();
	bool areContactsValid() const { return mContactsValid; }
	void setContactsValid(bool valid) { mContactsValid = valid; }
	bool contactArraysCaptured() const { return mContactArraysCaptured; }
	void setContactArraysCaptured(bool captured) { mContactArraysCaptured = captured; }
	ev4sio_physx::PxArray<ev4sio_physx::PxU32>& getGpuSnapshotIndices() { return mGpuSnapshotIndices; }

private:
	ev4sio_physx::PxScene& mScene;
	ev4sio_physx::PxU64 mFrameId;
	bool mFrameOpen;
	bool mContactsValid;
	bool mContactArraysCaptured;
	ev4sio_physx::PxArray<ev4sio_physx::PxU32> mGpuSnapshotIndices;

	ev4sio_physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicForce;
	ev4sio_physx::PxHashSet<const PxRigidDynamic*> mResetRigidDynamicTorque;

	ev4sio_physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksForce;
	ev4sio_physx::PxHashSet<const PxArticulationReducedCoordinate*> mResetArticulationLinksTorque;
};

}

class OmniPvdPxSampler : public ev4sio_physx::PxUserAllocated, public ev4sio_physx::PxErrorCallback
{
public:
	OmniPvdPxSampler();
	~OmniPvdPxSampler();
	// Stops sampling: flips the sampling flag off so subsequent SDK object
	// add/remove notifications and per-frame writes are suppressed. Calling it twice is
	// harmless. It is required before starting another recording, but optional for final
	// teardown: PxPhysics destruction emits final remove notifications before deleting the
	// sampler. A later snapshotAll() (driven by startSampling()) records the current state again.
	// Retain particle mirrors on stop; release excluded mirrors at the next snapshot and all on buffer teardown.
	// Primary-only snapshots also release diffuse mirrors.
	bool stopSampling();
	bool isSampling();
	bool isTakingSnapshot() const;
	// Emits a full snapshot of the current live simulation state to the currently
	// bound write stream, in referenced-before-referrer order: registers the full
	// schema + singletons, then walks shared resources -> shapes -> actors /
	// articulations / aggregates -> deformables / particles -> per-scene state. Called
	// from NpOmniPvd::startSampling() so the bound stream is self-contained (a full
	// state to attach to, followed by the changes after it). The writer's handle counters
	// were zeroed by OmniPvdWriter::setWriteStream when the stream was bound, so the schema
	// registration re-mints identical handles. Ensures sampling is on (emission flows via
	// onObjectAdd). Returns false if there is no sampler.
	bool snapshotAll();
	bool streamCaptureCatalog(bool initialize);
	static void getCaptureObjectIdentity(const ev4sio_physx::PxBase& object, const ev4sio_physx::PxScene*& scene,
		const ev4sio_physx::PxBase*& owner, const char*& name);
	bool setCaptureDesc(const ev4sio_physx::PxOmniPvdFilterDesc& desc);
	ev4sio_physx::PxU32 validateCaptureDesc(const ev4sio_physx::PxOmniPvdFilterDesc& desc) const;
	static void buildCaptureObjectCatalog(ev4sio_physx::PxHashMap<ev4sio_physx::PxU64, const ev4sio_physx::PxBase*>& catalog);
	void resetCaptureObjects(OmniPvdWriter& writer);
	void clearRetiredCaptureHandles();
	bool isCaptureHandleRetired(ev4sio_physx::PxU64 handle) const;
	bool isObjectRecorded(const ev4sio_physx::PxBase& object) const;
	bool isCapturingAllObjects() const;
	void prepareSceneJointState(const ev4sio_physx::PxScene& scene);
	void updateConstraintOmniPvdProperties(const ev4sio_physx::PxConstraint& constraint);
	// Skip connector updates during NpPhysics::createConstraint(): its owner is still initializing.
	void beginConstraintCreation(const ev4sio_physx::PxConstraintConnector& connector);
	void endConstraintCreation(const ev4sio_physx::PxConstraintConnector& connector);
	bool isJointStateActor(const ev4sio_physx::PxActor& actor) const;
	void storeJointActorPose(const ev4sio_physx::PxActor& actor, const ev4sio_physx::PxTransform& pose);
	void storeJointActorVelocity(const ev4sio_physx::PxActor& actor, const ev4sio_physx::PxVec3& velocity, bool angular);
	bool getJointActorPose(const ev4sio_physx::PxRigidActor& actor, ev4sio_physx::PxTransform& pose) const;
	bool getJointActorVelocity(const ev4sio_physx::PxRigidActor& actor, ev4sio_physx::PxVec3& linear, ev4sio_physx::PxVec3& angular) const;
	void streamArticulationReactions(OmniPvdWriter* writer, const ev4sio_physx::OmniPvdPxCoreRegistrationData* registrationData,
		const ev4sio_physx::PxArticulationReducedCoordinate& articulation);
#if PX_SUPPORT_GPU_PHYSX
	void beginSceneGpuReadback(const ev4sio_physx::PxScene& scene);
	bool streamDeformableState(OmniPvdWriter* writer, const ev4sio_physx::OmniPvdPxCoreRegistrationData* registrationData,
		const ev4sio_physx::PxDeformableBody& body);
	bool streamParticleBufferState(OmniPvdWriter* writer, const ev4sio_physx::OmniPvdPxCoreRegistrationData* registrationData,
		const ev4sio_physx::PxParticleBuffer& buffer);
	bool streamParticleSystemState(OmniPvdWriter* writer, const ev4sio_physx::OmniPvdPxCoreRegistrationData* registrationData,
		const ev4sio_physx::PxPBDParticleSystem& system);
#endif
	void setOmniPvdInstance(ev4sio_physx::NpOmniPvd* omniPvdIntance);

	// writes all contacts to the stream
	void streamSceneContacts(ev4sio_physx::NpScene& scene);

	static OmniPvdPxSampler* getInstance();
	static OmniPvdPxSampler* getSamplingInstance();

	void onObjectAdd(const ev4sio_physx::PxBase& object);
	// Batch variant: emits the object under a caller-owned write scope (the caller holds the
	// exclusive writer lock and provides the bound writer + registration data). snapshotAll()
	// uses this so a whole batch of objects shares one lock acquisition instead of one per object.
	// The single-object onObjectAdd(object) above opens exactly one scope and forwards here.
	void onObjectAdd(OmniPvdWriter* pvdWriter, const ev4sio_physx::OmniPvdPxCoreRegistrationData* pvdRegData, const ev4sio_physx::PxBase& object);
	void onObjectRemove(const ev4sio_physx::PxBase& object);
	void onConstraintActorsChange(const ev4sio_physx::PxConstraint& constraint);
#if PX_SUPPORT_GPU_PHYSX
	void onParticlePhaseChange(const ev4sio_physx::PxPBDParticleSystem& system);
#endif
	
	void reportAlreadySamplingError();
	virtual void reportError(ev4sio_physx::PxErrorCode::Enum code, const char* message, const char* file, int line) PX_OVERRIDE;
};


namespace ev4sio_physx
{

const OmniPvdPxCoreRegistrationData* NpOmniPvdGetPxCoreRegistrationData();
NpOmniPvd* NpOmniPvdGetInstance();

// True only while a recording is active. stopSampling() clears the flag; PxPhysics teardown
// deletes the sampler after emitting final remove notifications, which also makes this false. The
// OVD write macros gate on this so that no PhysX operation writes OVD data when not sampling. Used
// only inside PhysX core; the extensions write path gates on PxOmniPvd::isSampling() instead.
bool NpOmniPvdSampling();
bool NpOmniPvdRecordSecondaryData();
bool NpOmniPvdCaptureActor(const PxActor& actor);
bool NpOmniPvdCaptureActorState(const PxActor& actor);
bool NpOmniPvdCaptureArticulation(const PxArticulationReducedCoordinate& articulation);
bool NpOmniPvdCaptureScene(const PxScene& scene);
bool NpOmniPvdCaptureIsSelective();
void NpOmniPvdCaptureDependency(const void* object);
void NpOmniPvdCaptureRetireHandle(PxU64 handle);
bool NpOmniPvdShouldRecordCreation(const char* className, PxU64 objectHandle);
bool NpOmniPvdShouldRecordDestruction(const char* className, PxU64 objectHandle);
bool NpOmniPvdShouldRecordValue(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const char* className, const char* attributeName, PxU64 objectHandle);
bool NpOmniPvdShouldRecordAddition(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData,
	const char* className, const char* attributeName, PxU64 objectHandle, PxU64 valueHandle);
bool NpOmniPvdShouldRecordRemoval(OmniPvdWriter* writer, const OmniPvdPxCoreRegistrationData* registrationData, const char* className, const char* attributeName, PxU64 objectHandle, PxU64 valueHandle);

}

#endif

#endif
