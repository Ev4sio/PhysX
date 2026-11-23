// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseSdfShapeView.h"
#include "tensors/gpu/GpuSimulationData.h"

namespace omni
{
namespace ev4sio_physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuSdfShapeView : public BaseSdfShapeView
{
public:
    GpuSdfShapeView(GpuSimulationView* sim, const std::vector<SdfShapeEntry>& entries, int device);

    bool getSdfAndGradients(const TensorDesc* dstTensor, const TensorDesc* srcPointTensor) const override;

    ~GpuSdfShapeView() override;

private:
    GpuSimulationDataPtr mGpuSimData;

    int mDevice = -1;
    std::unordered_map<::ev4sio_physx::PxShapeGPUIndex, ::ev4sio_physx::PxU32> mShapeToViewIndexMap;

    // indexing data for all sdf shapes
    std::vector<GpuSdfShapeRecord> mSdfRecords;
    GpuSdfShapeRecord* mSdfRecordsDev = nullptr;

    // phyx shape indices
    ::ev4sio_physx::PxShapeGPUIndex* mShapeIndicesD;
    std::vector<::ev4sio_physx::PxShapeGPUIndex> mShapeIndices;

    // phyx sample points per shapes
    ::ev4sio_physx::PxU32* samplePointCountsD;
    std::vector<::ev4sio_physx::PxU32> samplePointCounts;

    ::ev4sio_physx::PxU32 mMaxNumPoints = 0;
    ::ev4sio_physx::PxVec4* samplePointsPerShape;
};
} // namespace tensors
} // namespace ev4sio_physx
} // namespace omni
