// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-SKIN-001
 * @covers AC-4
 */

#pragma once

#include "PxPhysicsAPI.h"
#include "PxDeformableSkinning.h"
#include "PxDeformableVolume.h"

#include <internal/Internal.h>

namespace omni
{
namespace ev4sio_physx
{

namespace deformables
{
    struct VolumeDeformableSkinningData
    {
        void packageSkinningData(::ev4sio_physx::PxTetmeshSkinningGpuData& packagedData);

        ::ev4sio_physx::PxDeformableVolume* mDeformableVolume;
        ::ev4sio_physx::PxU32* mGuideTetrahedraD;
        ::ev4sio_physx::PxTetrahedronMeshEmbeddingInfo* mSkinningEmbeddingInfoD;
        ::ev4sio_physx::PxVec3* mAllSkinnedVerticesD;
        ::ev4sio_physx::PxU32 mNumSkinnedVertices;
    };

    struct SurfaceDeformableSkinningData
    {
        void packageSkinningData(::ev4sio_physx::PxTrimeshSkinningGpuData& packagedData);

        ::ev4sio_physx::PxDeformableSurface* mDeformableSurface;
        ::ev4sio_physx::PxU32* mGuideTrianglesD;
        ::ev4sio_physx::PxU32 mNumGuideTriangles;
        ::ev4sio_physx::PxVec3* mGuideNormalsD;
        ::ev4sio_physx::PxU32 mNumGuideVertices;
        ::ev4sio_physx::PxTriangleMeshEmbeddingInfo* mSkinningEmbeddingInfoD;
        ::ev4sio_physx::PxVec3* mSkinnedVerticesD;
        ::ev4sio_physx::PxU32 mNumSkinnedVertices;
        ::ev4sio_physx::PxReal mHalfThickness;
    };

    // Base class for post-solve callback functionality of deformables.
    class DeformablePostSolveCallback : public ::ev4sio_physx::PxPostSolveCallback, public Allocateable
    {
    public:
        DeformablePostSolveCallback(CUstream stream, ::ev4sio_physx::PxCudaContextManager* cudaContextManager, ::ev4sio_physx::PxScene* scene);
        virtual ~DeformablePostSolveCallback();

        virtual void onPostSolve(CUevent startEvent) = 0;

        void synchronize();

        // Recorded after each skinning launch on the non-blocking mSkinningStream. A consumer on another
        // stream waits on it before gathering mAllSkinnedVerticesD, as the ovstage output read does for
        // REQ-READ-SKIN-001 AC-4. Before the first launch that wait is a no-op. Null only if creation
        // failed, and a consumer then has nothing to order against and must refuse.
        CUevent getSkinningDoneEvent() const { return mSkinningDoneEvent; }

        // The reverse edge of ADR-0008 Decision 7. A consumer calls this with its stream once its gather is
        // enqueued, and the next onPostSolve makes mSkinningStream wait for it before rewriting the buffer.
        // False if no event could be created or recorded, and the consumer then must not publish its gather.
        bool recordConsumerDone(::ev4sio_physx::PxCudaContext* cudaContext, CUstream consumerStream);

    protected:
        void recordSkinningDone(::ev4sio_physx::PxCudaContext* cudaContext);
        void waitForConsumers(::ev4sio_physx::PxCudaContext* cudaContext);

        CUstream mSkinningStream;
        CUevent mSkinningDoneEvent;
        // One event per pending consumer, recorded on the consumer's stream. The next skinning launch waits
        // for all of them before it rewrites the skinned vertices.
        std::vector<CUevent> mConsumerDoneEvents;
        std::vector<CUevent> mIdleConsumerDoneEvents;
        ::ev4sio_physx::PxCudaContextManager* mCudaContextManager;
        ::ev4sio_physx::PxDeformableSkinning* mSkinning;
        ::ev4sio_physx::PxScene* mScene;
        size_t mCurrentSize;        // Current size of the packaged skinning data buffers
        bool mNeedResize;           // For lazy allocation    
        bool isRegistered;          // Flag for registering the callback with the scene
    };

    class VolumeDeformablePostSolveCallback : public DeformablePostSolveCallback
    {
    public:
        VolumeDeformablePostSolveCallback(CUstream stream, ::ev4sio_physx::PxCudaContextManager* cudaContextManager, ::ev4sio_physx::PxScene* scene);
        ~VolumeDeformablePostSolveCallback();

        virtual void copySkinnedVerticesDtoHAsync(::ev4sio_physx::PxDeformableVolume* deformableVolume, ::ev4sio_physx::PxVec3* allSkinnedVerticesH);
        virtual void onPostSolve(CUevent startEvent);

        void addVolumeDeformableSkinningData(const VolumeDeformableSkinningData& skinningData);
        void removeVolumeDeformableSkinningData(const ::ev4sio_physx::PxDeformableVolume* deformableVolume);

    private:
        void resizeMemory(const size_t newSize);
        void releaseMemory();

        ::ev4sio_physx::PxTetmeshSkinningGpuData* mPackagedSkinningDataH;  // Pinned host memory for packaged skinning data
        ::ev4sio_physx::PxTetmeshSkinningGpuData* mPackagedSkinningDataD;  // Device memory for packaged skinning data

        std::vector<VolumeDeformableSkinningData> mSkinningData;
    };

    class SurfaceDeformablePostSolveCallback : public DeformablePostSolveCallback
    {
    public:
        SurfaceDeformablePostSolveCallback(CUstream stream, ::ev4sio_physx::PxCudaContextManager* cudaContextManager, ::ev4sio_physx::PxScene* scene);
        ~SurfaceDeformablePostSolveCallback();

        virtual void copySkinnedVerticesDtoHAsync(::ev4sio_physx::PxDeformableSurface* deformableSurface, ::ev4sio_physx::PxVec3* allSkinnedVerticesH);
        virtual void onPostSolve(CUevent startEvent);

        void addSurfaceDeformableSkinningData(const SurfaceDeformableSkinningData& skinningData);
        void removeSurfaceDeformableSkinningData(const ::ev4sio_physx::PxDeformableSurface* deformableSurface);

    private:
        void resizeMemory(const size_t newSize);
        void releaseMemory();

        ::ev4sio_physx::PxTrimeshSkinningGpuData* mPackagedSkinningDataH;  // Pinned host memory for packaged skinning data
        ::ev4sio_physx::PxTrimeshSkinningGpuData* mPackagedSkinningDataD;  // Device memory for packaged skinning data

        std::vector<SurfaceDeformableSkinningData> mSkinningData;
    };
} // namespace deformables
} // namespace ev4sio_physx
} // namespace omni
