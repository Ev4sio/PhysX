// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-OMNIPVD-CONTROL-001
// @covers AC-2 AC-3 AC-4 AC-5

// @implements REQ-OMNIPVD-CAPTURE-001
// @covers AC-1 AC-3 AC-4

// @implements REQ-ATTACH-SERVICES-001
// @covers AC-1 AC-2 AC-3 AC-4

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-4 AC-6 AC-8

/**
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-3
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 */

// @implements REQ-SDK-INIT-001
// @covers AC-1 AC-4

// @implements REQ-SDK-LIFECYCLE-002
// @covers AC-1 AC-3

#pragma once
#include <carb/tasking/TaskingUtils.h>

#include <PxPhysicsAPI.h>
#include <omnipvd/PxOmniPvd.h>
#include <carb/events/IEvents.h>
#include <carb/settings/ISettings.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include "utils/Pair.h"

#include <mutex>
#include <memory>
#include <unordered_set>

#include "PhysXScene.h"
#include "SceneMultiGPUMode.h"

class CarbPhysXErrorCallback;
class OmniPvdSocketWriteStream;
class OmniPvdWriteStream;

namespace cookingdataasync
{
class CookingDataAsync;
}

namespace omni
{
namespace ev4sio_physx
{
struct ICookingComputeService;
struct IPhysxFoundation;
typedef void* PhysxCookingAsyncContext;
} // namespace ev4sio_physx
} // namespace omni

namespace omni
{
namespace ev4sio_physx
{
// word3 flags for filtering shader
const uint32_t CONTACT_MODIFY_SURFACE_VELOCITY = 1 << 1;
const uint32_t CONTACT_SOLVE_DISABLE = 1 << 2;

const size_t kDefaultPhysXSceneId = 0;

using PhysXScenesMap = std::unordered_map<const ::ev4sio_physx::PxScene*, PhysXScene*>;

// Class holding and handling PhysX global objects like PxPhysics, PxFoundation
class PhysXSetup : private ::ev4sio_physx::PxOmniPvdEventCallback, private ::ev4sio_physx::PxOmniPvdCaptureFilter
{
public:
    PhysXSetup();
    ~PhysXSetup();

    ///////////////////////////////////////////////////////////////////////////////////////
    // Tolerances scales
    // Invalid stage units return an invalid scale and log an error; callers must reject it.
    ::ev4sio_physx::PxTolerancesScale getDefaultTolerances(double metersPerUnit);

    ///////////////////////////////////////////////////////////////////////////////////////
    // Cooking params
    ::ev4sio_physx::PxCookingParams getCookingParams(const ::ev4sio_physx::PxTolerancesScale& tolerances);
    const ::ev4sio_physx::PxCookingParams& getDefaultCookingParams()
    {
        return mDefaultCookingParams;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Pointer acceses
    // Lazily initializes the SDK; returns nullptr when initialization fails.
    ::ev4sio_physx::PxPhysics* getPhysics();
    // Acquire before realization, including empty candidates. Release only after
    // native objects and deferred work are retired, at attachment destruction.
    bool acquireAttachmentResources(usdparser::AttachedStage& attachment);
    ::ev4sio_physx::PxCudaContextManager* acquireAttachmentCudaContext(usdparser::AttachedStage& attachment);
    void releaseAttachmentResources(const usdparser::AttachedStage& attachment);
    bool hasResourceBorrowers() const;
    PhysxSharedSettings getSharedSettingsDefaults() const;
    ::ev4sio_physx::PxDefaultAllocator& getAllocator()
    {
        return mAllocator;
    }
    void reportCudaContextError(const usdparser::AttachedStage& owner, ::ev4sio_physx::PxCudaContextManager* manager);

    // Attachments on different threads (compiled cache, replicator) share one registry.
    ::ev4sio_physx::PxSerializationRegistry* getSerializationRegistry()
    {
        std::lock_guard<std::mutex> lock(mSerializationRegistryMutex);
        if (!mSerializationRegistry)
        {
            ::ev4sio_physx::PxPhysics* physics = getPhysics();
            if (!physics)
                return nullptr;
            mSerializationRegistry = ::ev4sio_physx::PxSerialization::createSerializationRegistry(*physics);
        }
        return mSerializationRegistry;
    }

    cookingdataasync::CookingDataAsync* getCookingDataAsync()
    {
        return mCookingDataAsync;
    }
    ICookingComputeService* getCookingComputeService();
    IPhysxCookingService* getCookingServiceInterface();
    IPhysxCookingServicePrivate* getCookingServicePrivateInterface();

    ///////////////////////////////////////////////////////////////////////////////////////
    // PxScene handling
    // Exact native-scene ownership lookup; foreign and retired scenes return null.
    PhysXScene* findPhysXScene(const ::ev4sio_physx::PxScene* scene) const
    {
        std::lock_guard<std::mutex> lock(mSceneRegistryMutex);
        const auto it = mPhysXScenes.find(scene);
        return it == mPhysXScenes.end() ? nullptr : it->second;
    }
    void releasePhysXScenes();
    // Native scene retirement only. The caller drains attachment services and releases
    // its records first; shared SDK/CUDA/dispatcher resources remain resident.
    void releasePhysXScenes(usdparser::AttachedStage& attachedStage);
    PhysXScene* createPhysXScene(usdparser::AttachedStage& attachedStage,
                                 size_t sceneId,
                                 double metersPerUnit,
                                 double kilogramsPerUnit,
                                 const usdparser::PhysxSceneDesc& sceneDesc);

    PhysXScenesMap getPhysXScenes() const
    {
        std::lock_guard<std::mutex> lock(mSceneRegistryMutex);
        return mPhysXScenes;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    // Thread count
    uint32_t getThreadCount() const
    {
        return mThreadCount;
    }
    void setThreadCount(uint32_t val)
    {
        mThreadCount = val;
    }
    ///////////////////////////////////////////////////////////////////////////////////////
    // PVD
    // Recording state belongs to the shared PVD transport, independent of owner clocks.
    void notePvdSimulation() { mPvdHasSimulation.store(true); }
    void connectPVD();
    void disconnectPVD();
    ::ev4sio_physx::PxPvd* getPvd()
    {
        return mVisualDebugger;
    }
    ::ev4sio_physx::PxOmniPvd* getOmniPvd()
    {
        return mOmniPvd;
    }
    // Runtime change of PVD settings - CAVEAT: this must NOT be called in a running simulation
    // (no synchronization is provided between post-frame-end pvd flushing and client reconnection)
    void changePVDSettings(bool enableLegacyPVD);

    // carb.settings subscriptions owned by this object (the legacy PVD switch, kSettingPVDEnabled,
    // routed to changePVDSettings). The started runtime owns their lifetime: OmniPhysX::onStartup() subscribes and
    // OmniPhysX::onShutdown() unsubscribes, next to OmniPhysX's own subscriptions. The destructor
    // is a backstop, not the release point: the OmniPhysX instance holding this member is not
    // destroyed at shutdown, while carb.settings outlives it and keeps dispatching to every
    // subscription it still holds -- into this retired object after a restart, or into unmapped
    // code once the module that held the callback is unloaded.
    void subscribeToSettingsChangeEvents(); // no-op while already subscribed
    void unsubscribeFromSettingsChangeEvents(); // idempotent, safe without a prior subscribe
    size_t getSettingsSubscriptionCount() const
    {
        return mSubscribedSettings.size();
    }

    OmniPvdRecordingResult startOmniPvdRecording(
        const OmniPvdDestination& destination,
        const ::ev4sio_physx::PxOmniPvdFilterDesc* capture = nullptr,
        AttachHandle selectionOwner = omni::physics::kNoAttach, bool discovery = false);
    OmniPvdRecordingResult stopOmniPvdRecording();
    bool isOmniPvdRecording() const;
    bool isOmniPvdSecondaryDataEnabled() const
    {
        return mOmniPvdRecordSecondaryData;
    }
    AttachHandle getOmniPvdCaptureOwner() const { return mOmniPvdCaptureOwner.load(std::memory_order_acquire); }
    bool shouldProcessOmniPvdCaptureRequests(AttachHandle attach) const
    {
        if (!mOmniPvdCaptureControlEnabled.load(std::memory_order_acquire))
            return false;
        const AttachHandle owner = getOmniPvdCaptureOwner();
        return owner == attach || owner == omni::physics::kNoAttach;
    }
    bool hasPendingOmniPvdCaptureRequest(AttachHandle attach);
    void processOmniPvdCaptureRequests(AttachHandle attach);

    inline const ::ev4sio_physx::PxVehiclePvdAttributeHandles* getVehiclePvdRegistrationHandles() const
    {
        return mVehiclePvdRegistrationHandles;
    }

    ::ev4sio_physx::PxConvexMesh* getCylinderConvexMesh(omni::ev4sio_physx::usdparser::Axis axis) const;
    ::ev4sio_physx::PxConvexMesh* getConeConvexMesh(omni::ev4sio_physx::usdparser::Axis axis) const;
    ::ev4sio_physx::PxConvexMesh* getVehicleWheelCylinderConvexMesh(omni::ev4sio_physx::usdparser::Axis axis) const;

    ///////////////////////////////////////////////////////////////////////////////////////
    // If any scene is async return true
    bool isAsyncSimEnabled() const
    {
        std::lock_guard<std::mutex> registry(mSceneRegistryMutex);
        for (PhysXScenesMap::const_reference ref : mPhysXScenes)
        {
            if (ref.second->getUpdateType() == usdparser::eAsynchronous)
                return true;
        }
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // create base PhysX classses
    // A.B. this should be optimized, we dont have to create this with every simulation start
    void createPhysics(const ::ev4sio_physx::PxTolerancesScale& tolerances);

    bool hasBeenInitiallyCreated() const
    {
        return mFoundation != nullptr;
    }
    void createPhysics(); // initial create
    void releasePhysics(); // final release, should be called when app is closing

    void resetPhysXErrorCounter();
    void setMaxNumberOfPhysXErrors(uint32_t maxNumberOfPhysXErrors);

private:
    void initializeCookingServiceInterfaces();
    void releaseCookingService();
    void createCookingAsyncContext();
    void releaseCookingAsyncContext();
    void cleanupPhysics(); // called with scene destruction, this should not be required in the end
    bool releaseOmniPvdWriteStream();
    bool closeOmniPvdWriteStream();
    void onStartSampling(::ev4sio_physx::PxOmniPvd& omniPvd, const ::ev4sio_physx::PxOmniPvdFilterDesc* filter) override;
    void onCaptureReset(::ev4sio_physx::PxOmniPvd& omniPvd, const ::ev4sio_physx::PxOmniPvdFilterDesc& filter) override;
    bool filterCaptureRequest(::ev4sio_physx::PxOmniPvdFilterDesc& filter) override;
    bool restrictOmniPvdCaptureToOwner(::ev4sio_physx::PxOmniPvdFilterDesc& filter, AttachHandle owner,
                                     std::vector<const ::ev4sio_physx::PxScene*>& sceneStorage, bool expandEmpty = true);
    void registerVehiclePvdCallback();
    void unregisterVehiclePvdCallback();
    void releaseVehiclePvdRegistrationHandles();
    void releaseVehiclePvdTelemetry();
    ::ev4sio_physx::PxFoundation* mFoundation{ nullptr };
    ::ev4sio_physx::PxPhysics* mPhysics{ nullptr };
    std::unordered_set<const usdparser::AttachedStage*> mAttachmentBorrowers;
    ::ev4sio_physx::PxSerializationRegistry* mSerializationRegistry{ nullptr };
    std::mutex mSerializationRegistryMutex;
    carb::settings::ISettings* mISettings{ nullptr };

    ::ev4sio_physx::PxDefaultAllocator mAllocator;
    CarbPhysXErrorCallback* mErrorCallback{ nullptr };

    ::ev4sio_physx::PxTolerancesScale mDefaultScale;
    ::ev4sio_physx::PxCookingParams mDefaultCookingParams{ mDefaultScale };
    cookingdataasync::CookingDataAsync* mCookingDataAsync{ nullptr };
    omni::ev4sio_physx::ICookingComputeService* mCookingComputeService{ nullptr };
    omni::ev4sio_physx::IPhysxCookingServicePrivate mCookingServicePrivate{};
    omni::ev4sio_physx::IPhysxCookingService mCookingService{};
    omni::ev4sio_physx::PhysxCookingAsyncContext mCookingServiceContext{ nullptr };
    bool mPhysicsReleased{ false };
    omni::ev4sio_physx::IPhysxFoundation* mPhysxFoundation{ nullptr };

    ::ev4sio_physx::PxPvd* mVisualDebugger{ nullptr };
    ::ev4sio_physx::PxPvdTransport* mPvdTransport{ nullptr };
    std::atomic<bool> mPvdHasSimulation{ false };
    ::ev4sio_physx::PxOmniPvd* mOmniPvd{ nullptr };
    enum class OmniPvdStreamKind : uint8_t
    {
        eNone,
        eRuntimeFile,
        eTcp,
    };
    OmniPvdWriteStream* mOmniPvdWriteStream{ nullptr };
    OmniPvdStreamKind mOmniPvdStreamKind{ OmniPvdStreamKind::eNone };
    std::atomic<AttachHandle> mOmniPvdCaptureOwner{ omni::physics::kNoAttach };
    std::atomic<bool> mOmniPvdCaptureControlEnabled{ false };
    std::vector<const ::ev4sio_physx::PxScene*> mOmniPvdCaptureScenes;
    ::ev4sio_physx::PxVehiclePvdAttributeHandles* mVehiclePvdRegistrationHandles{ nullptr };
    bool mVehiclePvdSnapshotSucceeded{ false };
    bool mOmniPvdRecordSecondaryData{ true };

    uint32_t mThreadCount{ 8 };

    struct AttachmentCudaResource
    {
        int mode;
        int ordinal;
        uintptr_t externalContext;
        bool synchronous;
        std::shared_ptr<::ev4sio_physx::PxCudaContextManager> manager;
    };
    // PhysX caches GPU kernel wranglers by manager address until PxPhysics dies.
    // Retain compatible managers for that whole SDK generation, including gaps
    // between attachments, so an address cannot alias a stale kernel cache entry.
    std::vector<AttachmentCudaResource> mAttachmentCudaResources;
    mutable carb::tasking::RecursiveMutexWrapper mResourceMutex;
    // Native PhysX query callbacks look up scenes inside thread-local API-check scopes.
    // This lock must not suspend or migrate the calling task. Keep its critical sections
    // limited to registry access; never call task-aware locks or callbacks while holding it.
    mutable std::mutex mSceneRegistryMutex;
    bool mReleasingPhysXScenes{ false };
    bool mExtensionsInitialized{ false };
    bool mVehicleSDKInitialized{ false }; // guards against multiple de-init, since scene release can be called
                                          // more than once (TODO: investigate why); also guards multiple init,
                                          // though that hasn't been observed.
    PhysXScenesMap mPhysXScenes;

    mutable ::ev4sio_physx::PxConvexMesh* mCylinderMeshX{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mCylinderMeshY{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mCylinderMeshZ{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mConeMeshX{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mConeMeshY{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mConeMeshZ{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mVehicleWheelCylinderMeshX{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mVehicleWheelCylinderMeshY{ nullptr };
    mutable ::ev4sio_physx::PxConvexMesh* mVehicleWheelCylinderMeshZ{ nullptr };

    // Subscriptions to settings changes
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;
};

} // namespace ev4sio_physx
} // namespace omni
