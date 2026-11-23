// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2 AC-4

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-4

// @implements REQ-CACHE-ATTACH-001
// @covers AC-3

/**
 * @implements REQ-SIM-MULTISCENE-001
 * @covers AC-2
 *
 * @implements REQ-READ-SDF-001
 * @covers AC-7
 */

#pragma once

#include "internal/InternalScene.h"

#include <atomic>
#include <vector>

#include <carb/tasking/TaskingUtils.h>

namespace omni
{
namespace ev4sio_physx
{
namespace usdparser
{
class AttachedStage;
}
class PhysXSetup;
class OmniPhysX;
class PhysXScene;
class ContactReport;

class VehicleUpdateTask;

class OmniFilterCallback : public ::ev4sio_physx::PxSimulationFilterCallback
{
    // PT: the old API uses PxU32s for pairIDs. We can just re-route that to the new API and keep supporting both.

    virtual ::ev4sio_physx::PxFilterFlags pairFound(::ev4sio_physx::PxU32 pairID,
                                             ::ev4sio_physx::PxFilterObjectAttributes attributes0,
                                             ::ev4sio_physx::PxFilterData filterData0,
                                             const ::ev4sio_physx::PxActor* a0,
                                             const ::ev4sio_physx::PxShape* s0,
                                             ::ev4sio_physx::PxFilterObjectAttributes attributes1,
                                             ::ev4sio_physx::PxFilterData filterData1,
                                             const ::ev4sio_physx::PxActor* a1,
                                             const ::ev4sio_physx::PxShape* s1,
                                             ::ev4sio_physx::PxPairFlags& pairFlags);

    virtual void pairLost(::ev4sio_physx::PxU32 /*pairID*/,
                          ::ev4sio_physx::PxFilterObjectAttributes /*attributes0*/,
                          ::ev4sio_physx::PxFilterData /*filterData0*/,
                          ::ev4sio_physx::PxFilterObjectAttributes /*attributes1*/,
                          ::ev4sio_physx::PxFilterData /*filterData1*/,
                          bool /*objectRemoved*/)
    {
    }

    virtual bool statusChange(::ev4sio_physx::PxU32& /*pairID*/, ::ev4sio_physx::PxPairFlags& /*pairFlags*/, ::ev4sio_physx::PxFilterFlags& /*filterFlags*/)
    {
        return false;
    }

    // PT: the new API uses PxU64s for pairIDs

    virtual ::ev4sio_physx::PxFilterFlags pairFound(::ev4sio_physx::PxU64 pairID,
                                             ::ev4sio_physx::PxFilterObjectAttributes attributes0,
                                             ::ev4sio_physx::PxFilterData filterData0,
                                             const ::ev4sio_physx::PxActor* a0,
                                             const ::ev4sio_physx::PxShape* s0,
                                             ::ev4sio_physx::PxFilterObjectAttributes attributes1,
                                             ::ev4sio_physx::PxFilterData filterData1,
                                             const ::ev4sio_physx::PxActor* a1,
                                             const ::ev4sio_physx::PxShape* s1,
                                             ::ev4sio_physx::PxPairFlags& pairFlags);

    virtual void pairLost(::ev4sio_physx::PxU64 /*pairID*/,
                          ::ev4sio_physx::PxFilterObjectAttributes /*attributes0*/,
                          ::ev4sio_physx::PxFilterData /*filterData0*/,
                          ::ev4sio_physx::PxFilterObjectAttributes /*attributes1*/,
                          ::ev4sio_physx::PxFilterData /*filterData1*/,
                          bool /*objectRemoved*/)
    {
    }

    virtual bool statusChange(::ev4sio_physx::PxU64& /*pairID*/, ::ev4sio_physx::PxPairFlags& /*pairFlags*/, ::ev4sio_physx::PxFilterFlags& /*filterFlags*/)
    {
        return false;
    }

public:
    bool mInvertedCollisionFilter{ false };
    const OmniPhysX* mOmniPhysX{ nullptr };
    ContactReport* mContactReport{ nullptr };
};

class OmniContactReportCallback : public ::ev4sio_physx::PxSimulationEventCallback, public ::ev4sio_physx::PxContactModifyCallback
{
    virtual void onConstraintBreak(::ev4sio_physx::PxConstraintInfo* constraints, ::ev4sio_physx::PxU32 count) final;
    virtual void onWake(::ev4sio_physx::PxActor** actors, ::ev4sio_physx::PxU32 count) final;
    virtual void onSleep(::ev4sio_physx::PxActor** actors, ::ev4sio_physx::PxU32 count) final;
    virtual void onTrigger(::ev4sio_physx::PxTriggerPair* pairs, ::ev4sio_physx::PxU32 count) final;
    virtual void onAdvance(const ::ev4sio_physx::PxRigidBody* const* bodyBuffer,
                           const ::ev4sio_physx::PxTransform* poseBuffer,
                           const ::ev4sio_physx::PxU32 count) final;
    virtual void onContact(const ::ev4sio_physx::PxContactPairHeader& pairHeader,
                           const ::ev4sio_physx::PxContactPair* pairsIn,
                           ::ev4sio_physx::PxU32 countIn) final;
    virtual void onContactModify(::ev4sio_physx::PxContactModifyPair* pairs, ::ev4sio_physx::PxU32 count) final;

    bool isFilteredEvent(const ::ev4sio_physx::PxShape* triggerShape, const ::ev4sio_physx::PxShape* otherShape);

public:
    OmniPhysX* mOmniPhysX{ nullptr };
    PhysXScene* mPhysXScene{ nullptr };
    ContactReport* mContactReport{ nullptr };

    float mStepSize{ 1.0f / 60.0f };
};

class VehicleUpdateSyncTask : public ::ev4sio_physx::PxLightCpuTask
{
private:
    VehicleUpdateSyncTask();

public:
    VehicleUpdateSyncTask(::ev4sio_physx::PxSync& sync) : mSync(sync)
    {
    }

    virtual ~VehicleUpdateSyncTask()
    {
    }

    virtual void run()
    {
    }

    virtual void release()
    {
        // no need to call release() of base class as there is no continuation task
        mSync.set();
    }

    virtual const char* getName() const
    {
        return "PhysXVehicleUpdateSync";
    }

private:
    ::ev4sio_physx::PxSync& mSync;
};

class SimulationCompletionTask : public ::ev4sio_physx::PxLightCpuTask
{
public:
    void initialize(carb::tasking::ITasking* tasking, carb::tasking::TaskContext taskContext)
    {
        mTasking = tasking;
        mTaskContext = taskContext;
    }

    virtual void run()
    {
    }

    virtual void release()
    {
        carb::tasking::ITasking* const tasking = mTasking;
        const carb::tasking::TaskContext taskContext = mTaskContext;
        ::ev4sio_physx::PxLightCpuTask::release();
        tasking->wakeTask(taskContext);
    }

    virtual const char* getName() const
    {
        return "PhysXSimulationCompletion";
    }

private:
    carb::tasking::ITasking* mTasking{ nullptr };
    carb::tasking::TaskContext mTaskContext{ carb::tasking::kInvalidTaskContext };
};

class PhysXStepper : public ::ev4sio_physx::PxLightCpuTask
{
public:
    PhysXStepper();

    virtual ~PhysXStepper();

    float getStepSize() const
    {
        return mStepSize;
    }

    void launch(int nbSubsteps,
                float stepSize,
                PhysXScene* scene,
                ::ev4sio_physx::PxTaskManager* taskManager,
                bool substeppingEnabled,
                bool forceAsync,
                bool lastStep);

    void updateForces();
    void updateQuasistatic();
    void updateVehicles();
    void updateParticles(int step, int numSteps);
    void updateDeformables(int step, int numSteps);

    virtual void run();

    void waitForCompletion(bool sendPostEvents = true);

    bool isComplete() const
    {
        return mComplete;
    }

    virtual const char* getName() const
    {
        return "PhysXStepper";
    }

    static void customizeVehicleSimulationContext(::ev4sio_physx::PxVehiclePhysXSimulationContext&,
                                                  const PhysXActorVehicleBase&,
                                                  PhysXSetup&);

private:
    void createVehicleUpdateTasks();
    void updateQuasistaticActors(const ::ev4sio_physx::PxU32 numActors, ::ev4sio_physx::PxActor** actors);

    carb::tasking::MutexWrapper mCompleteLock;
    carb::tasking::ConditionVariableWrapper mCompleteCV;

    bool mComplete;
    int mNbSubsteps;
    float mStepSize;
    PhysXScene* mPhysXScene;
    AttachHandle mStepAttach = omni::physics::kNoAttach;
    bool mUpdateVelocities;
    bool mForceAsync;
    bool mSendStepping;
    bool mLastStep;
    bool mSubsteppingEnabled;
    bool mSkipSimulation;
    ::ev4sio_physx::PxSceneQueryUpdateMode::Enum mSceneQueryUpdateMode;
    SimulationCompletionTask mSimulationCompletionTask;

    // vehicle update
    std::vector<VehicleUpdateTask*> mVehicleUpdateTasks;
    // mVehicleSync must be declared (and therefore constructed) before
    // mVehicleUpdateSyncTask, which binds a reference to it in its
    // constructor -- member init order follows declaration order.
    ::ev4sio_physx::PxSync mVehicleSync;
    VehicleUpdateSyncTask mVehicleUpdateSyncTask;
    std::atomic<int32_t> mVehicleProcessingIndex;
};


class PhysXScene
{
public:
    ::ev4sio_physx::PxScene* getScene() const
    {
        return mScene;
    }

    internal::InternalScene* getInternalScene() const
    {
        return mInternalScene;
    }

    omni::physics::parse::ObjectKey getSceneSdfPath() const
    {
        return mSceneSdfPath;
    }

    ::ev4sio_physx::PxControllerManager* getControllerManager() const
    {
        return mControllerManager;
    }

    ::ev4sio_physx::PxMaterial* getDefaultMaterial() const
    {
        return mMaterial;
    }
    void resetDefaultMaterial();
    omni::physics::parse::ObjectKey getDefaultMaterialPath() const
    {
        return mMaterialPath;
    }

    ::ev4sio_physx::PxDeformableVolumeMaterial* getDefaultVolumeDeformableMaterial() const
    {
        return mVolumeDeformableMaterial;
    }
    omni::physics::parse::ObjectKey getDefaultVolumeDeformableMaterialPath() const
    {
        return mVolumeDeformableMaterialPath;
    }

    ::ev4sio_physx::PxDeformableSurfaceMaterial* getDefaultSurfaceDeformableMaterial() const
    {
        return mSurfaceDeformableMaterial;
    }
    omni::physics::parse::ObjectKey getDefaultSurfaceDeformableMaterialPath() const
    {
        return mSurfaceDeformableMaterialPath;
    }

    ::ev4sio_physx::PxPBDMaterial* getDefaultPBDMaterial() const
    {
        return mPBDMaterial;
    }
    omni::physics::parse::ObjectKey getDefaultPBDMaterialPath() const
    {
        return mPBDMaterialPath;
    }

    uint32_t getTimeStepsPerSeconds() const
    {
        return mTimeStepsPerSeconds;
    }
    void setTimeStepsPerSeconds(uint32_t val)
    {
        mTimeStepsPerSeconds = val;
    }

    usdparser::SceneUpdateType getUpdateType() const
    {
        return mSceneUpdateType;
    }
    void setUpdateType(usdparser::SceneUpdateType type)
    {
        mSceneUpdateType = type;
    }

    bool isReadbackSuppressed() const
    {
        return mScene && mScene->getFlags().isSet(::ev4sio_physx::PxSceneFlag::eENABLE_DIRECT_GPU_API);
    }

    bool isInvertedCollisionGroupFilter() const
    {
        return mInvertedCollisionGroupFilter;
    }

    OmniContactReportCallback& getContactReportCallback()
    {
        return mContactReportCallback;
    }

    bool isSceneQuerySupported() const
    {
        return mSupportSceneQuery;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Contact report
    const ContactReport* getContactReport() const
    {
        return mContactReport;
    }
    ContactReport* getContactReport()
    {
        return mContactReport;
    }

    ///////////////////////////////////////////////////////////////////////////////////////

    void waitForCompletion(bool doPostWork = true)
    {
        mPhysXStepper.waitForCompletion(doPostWork);
    }

    bool isComplete() const
    {
        return mPhysXStepper.isComplete();
    }

    void computeSubstepping(float elapsedSecs, float fixedTimeStep, float timestepsPerSecond);
    void setSubstepping(uint32_t steps, float elapsedSecs)
    {
        mRemaining = 0.0f;
        mCurrentStep = steps;
        mCurrentTimeStep = elapsedSecs;
        if (steps > 1)
        {
            mSubsteppingEnabled = true;
        }
        else
        {
            mSubsteppingEnabled = false;
        }
    }

    void launch(bool forceAsync);
    void step();

    void decreaseCurrentStep()
    {
        CARB_ASSERT(mCurrentStep > 0);
        mCurrentStep--;
    }
    uint32_t getCurrentStep() const
    {
        return mCurrentStep;
    }
    float getCurrentTimeStep() const
    {
        return mCurrentTimeStep;
    }

    void updateMirroredBodies();

    ::ev4sio_physx::PxSceneQueryUpdateMode::Enum getSceneQueryUpdateMode() const
    {
        return mSceneQueryUpdateMode;
    }

    static PhysXScene* createPhysXScene(usdparser::AttachedStage& attachedStage,
                                        size_t sceneId,
                                        PhysXSetup&,
                                        double metersPerUnit,
                                        double kilogramsPerUnit,
                                        const usdparser::PhysxSceneDesc& sceneDesc);

    bool isNewScene() const
    {
        return mNewScene;
    }

    usdparser::AttachedStage& getAttachedStage() const
    {
        return mAttachedStage;
    }

    // The attachment's record version as of this scene's latest PxScene::simulate(), 0 before the first. PhysX
    // uploads an object to the GPU only in the simulate that follows its insertion, so a later version means
    // objects the device has not seen yet. The stepper and a simulation view's step both note it just before
    // they simulate.
    uint64_t recordVersionAtLastSimulate() const
    {
        return mRecordVersionAtLastSimulate.load(std::memory_order_relaxed);
    }
    void noteSimulateLaunch();

    bool isFullGpuPipelineAvailable()
    {
        if (!mScene)
            return false;

        ::ev4sio_physx::PxSceneFlags sceneFlags = mScene->getFlags();
        ::ev4sio_physx::PxBroadPhaseType::Enum bpType = mScene->getBroadPhaseType();
        return (sceneFlags & ::ev4sio_physx::PxSceneFlag::eENABLE_GPU_DYNAMICS && bpType == ::ev4sio_physx::PxBroadPhaseType::eGPU);
    }

private:
    friend class PhysXSetup;
    PhysXScene(usdparser::AttachedStage& attachedStage);
    ~PhysXScene();

    usdparser::AttachedStage& mAttachedStage;
    ::ev4sio_physx::PxScene* mScene;
    ::ev4sio_physx::PxControllerManager* mControllerManager;
    internal::InternalScene* mInternalScene;
    uint32_t mTimeStepsPerSeconds;
    usdparser::SceneUpdateType mSceneUpdateType;
    bool mInvertedCollisionGroupFilter;
    bool mSupportSceneQuery;
    omni::physics::parse::ObjectKey mSceneSdfPath;

    // default materials
    ::ev4sio_physx::PxMaterial* mMaterial;
    omni::physics::parse::ObjectKey mMaterialPath;
    ::ev4sio_physx::PxDeformableVolumeMaterial* mVolumeDeformableMaterial;
    omni::physics::parse::ObjectKey mVolumeDeformableMaterialPath;
    ::ev4sio_physx::PxDeformableSurfaceMaterial* mSurfaceDeformableMaterial;
    omni::physics::parse::ObjectKey mSurfaceDeformableMaterialPath;
    ::ev4sio_physx::PxPBDMaterial* mPBDMaterial;
    omni::physics::parse::ObjectKey mPBDMaterialPath;

    OmniFilterCallback mFilterCallback;
    OmniContactReportCallback mContactReportCallback;

    ::ev4sio_physx::PxSceneQueryUpdateMode::Enum mSceneQueryUpdateMode;

    // simulation stepping
    float mRemaining;
    bool mSubsteppingEnabled;
    uint32_t mCurrentStep;
    float mCurrentTimeStep;
    PhysXStepper mPhysXStepper;
    bool mNewScene;
    std::atomic<uint64_t> mRecordVersionAtLastSimulate{ 0 };

    // Contact report
    ContactReport* mContactReport;
};

bool isRigidBodyDynamic(const omni::ev4sio_physx::usdparser::AttachedStage& attachedStage, omni::ev4sio_physx::usdparser::ObjectId id);
// Whether a scene of the attachment can simulate on the GPU: false when its GPU pipeline
// override selects the CPU, or the process runs in CPU mode or lacks the CUDA library.
bool attachmentMayUseGpu(const usdparser::AttachedStage& attachedStage);
// Whether the scene created from `desc` asks for a CUDA context: the attachment may use the
// GPU, and its GPU pipeline override selects the GPU, or leaves the choice to the scene,
// which enables GPU dynamics or the GPU broadphase.
bool sceneRequestsGpu(const usdparser::AttachedStage& attachedStage, const usdparser::PhysxSceneDesc& desc);


} // namespace ev4sio_physx
} // namespace omni
