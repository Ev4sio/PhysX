// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-TENSOR-CONTACT-004
// @covers AC-1 AC-2

/**
 * @implements REQ-TENSOR-CONTACT-003
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-6
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-10
 *
 * @implements REQ-READ-SDF-001
 * @covers AC-3
 *
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-7 AC-14 AC-20 AC-21
 */

#pragma once

#include "tensors/ForceComponent.h"
#include "tensors/PhysicsTypes.h"
#include "tensors/gpu/CudaCommon.h"
#include "tensors/gpu/GpuSimulationData.h"

// Included rather than forward-declared: the gather takes a DofScalePolicy by value, and the tendon write a
// TendonFieldWrite. The headers are dependency-free (cstdint plus a host/device macro) so the CUDA side can include
// them.
#include "tensors/ArticulationDofOvStageRecord.h"
#include "tensors/ArticulationLinkOvStageRecord.h"
#include "tensors/ArticulationTendonOvStageRecord.h"

namespace omni
{
namespace ev4sio_physx
{
namespace tensors
{
struct GpuArticulationRootRecord;
struct GpuArticulationDofRecord;
struct ArticulationTendonOvStageRecord;
struct GpuArticulationLinkRecord;
struct GpuArticulationFixedTendonRecord;
struct GpuArticulationSpatialTendonRecord;
struct GpuRigidContactFilterIdPair;
struct RigidContactReferentTables;
struct GpuRigidBodyRecord;
struct GpuPointInstancerRecord;
struct GpuPointSetReadRecord;
struct PointSetTransform;
enum class PointSetSourceLayout : uint32_t;
struct SingleAllocPolicy;

// Clear caller-owned contact storage in the same stream as the contact kernels.
bool clearContactBuffer(void* destination, size_t bytes);

//
// articulations
//
::ev4sio_physx::PxU32 fillArtiDirtyIndices(SingleAllocPolicy& policy,
                                    ::ev4sio_physx::PxU32* indicesRet,
                                    const ::ev4sio_physx::PxU32* allIndicies,
                                    const ArticulationGpuFlags* artiDirtyFlags,
                                    const ArticulationGpuFlag::Enum flag,
                                    const ::ev4sio_physx::PxU32 numArtis);

::ev4sio_physx::PxU32 fillArtiTransforms(SingleAllocPolicy& policy,
                                  ::ev4sio_physx::PxTransform* transformDev,
                                  ::ev4sio_physx::PxU32* indicesRet,
                                  const ::ev4sio_physx::PxU32* allIndicies,
                                  const ArticulationGpuFlags* artiDirtyFlags,
                                  const ArticulationGpuFlag::Enum flag,
                                  const ::ev4sio_physx::PxU32 numArtis);

::ev4sio_physx::PxU32 fillArtiVelocities(SingleAllocPolicy& policy,
                                  ::ev4sio_physx::PxVec3* artLinVelDev,
                                  ::ev4sio_physx::PxVec3* artAngVelDev,
                                  ::ev4sio_physx::PxU32* indicesRet,
                                  const ::ev4sio_physx::PxU32* allIndicies,
                                  const ArticulationGpuFlags* artiDirtyFlags,
                                  const ArticulationGpuFlag::Enum flag,
                                  const ::ev4sio_physx::PxU32 numArtis);

::ev4sio_physx::PxU32 fillArtiFT(SingleAllocPolicy& policy,
                          ::ev4sio_physx::PxVec3* FT,
                          ::ev4sio_physx::PxU32* indicesRet,
                          const ::ev4sio_physx::PxU32* allIndicies,
                          const ArticulationGpuFlags* artiDirtyFlags,
                          ArticulationGpuFlags* artiLinksDirtyFlags,
                          const ArticulationGpuFlag::Enum flag,
                          const ::ev4sio_physx::PxU32 numArtis,
                          const ::ev4sio_physx::PxU32 maxLinks);

bool fetchArtiRootTransforms(TensorTransform* dst,
                             const ::ev4sio_physx::PxTransform* src,
                             const ::ev4sio_physx::PxU32 numArti,
                             const GpuArticulationRootRecord* rootRecords);

bool submitArtiRootTransforms(::ev4sio_physx::PxTransform* dst,
                              const TensorTransform* src,
                              const ::ev4sio_physx::PxU32* srcIndices,
                              ::ev4sio_physx::PxU32* dirtyArtiGpuIndices,
                              const ::ev4sio_physx::PxU32 numIndices,
                              const ::ev4sio_physx::PxU32 numArti,
                              const GpuArticulationRootRecord* rootRecords);

bool fetchArtiRootVelocities(TensorVelAcc* dst,
                             const ::ev4sio_physx::PxVec3* srcLin,
                             const ::ev4sio_physx::PxVec3* srcAng,
                             const ::ev4sio_physx::PxU32 numArti);

bool fetchArtiRootPoseColumnOvStage(float* dst, const ::ev4sio_physx::PxTransform* src, ::ev4sio_physx::PxU32 count, bool orientation,
                                    cudaStream_t stream);

bool fetchArtiRootVelocityColumnOvStage(float* dst, const ::ev4sio_physx::PxVec3* src, ::ev4sio_physx::PxU32 count,
                                        cudaStream_t stream);

// In-place subspace-origin subtraction over a whole world-COM column. Distinct from the
// point-instancer reframe (InstancerReframe.h), which is a full affine composition with rotation.
bool applySubspaceOriginArtiMassCentersOvStage(::ev4sio_physx::PxVec3* dst,
                                               ::ev4sio_physx::PxU32 count,
                                               const GpuArticulationRootRecord* rootRecords);

bool submitArtiRootVelocities(::ev4sio_physx::PxVec3* dstLin,
                              ::ev4sio_physx::PxVec3* dstAng,
                              const TensorVelAcc* src,
                              const ::ev4sio_physx::PxU32* srcIndices,
                              ::ev4sio_physx::PxU32* dirtyArtiGpuIndices,
                              const ::ev4sio_physx::PxU32 numIndices,
                              const ::ev4sio_physx::PxU32 numArti,
                              const GpuArticulationRootRecord* rootRecords);

bool fetchArtiLinkTransforms(TensorTransform* dst,
                             const ::ev4sio_physx::PxTransform* src,
                             const ::ev4sio_physx::PxU32 numLinks,
                             const ::ev4sio_physx::PxU32 maxLinks,
                             const ::ev4sio_physx::PxU32 simMaxLinks,
                             const GpuArticulationLinkRecord* linkRecords);

// fetches link velocities or accelerations from src
bool fetchArtiLinkVelocitiesAccelerations(TensorVelAcc* dst,
                                          const ::ev4sio_physx::PxVec3* linSrc,
                                          const ::ev4sio_physx::PxVec3* angSrc,
                                          const ::ev4sio_physx::PxU32 numLinks,
                                          const ::ev4sio_physx::PxU32 maxLinks,
                                          const ::ev4sio_physx::PxU32 simMaxLinks);

bool fetchArtiMassMatrices(float* dst,
                           const float* src,
                           const ::ev4sio_physx::PxU32 numElements,
                           const ::ev4sio_physx::PxU32 massMatrixSize,
                           const ::ev4sio_physx::PxU32 simMassMatrixSize,
                           const ::ev4sio_physx::PxU32 generalizedCoords,
                           const ::ev4sio_physx::PxU32 rootDofs,
                           const ::ev4sio_physx::PxU32 dofRecordBase,
                           const GpuArticulationDofRecord* dofRecords,
                           bool applyBodyOrderSign,
                           cudaStream_t stream = nullptr);

// Cohort gather for the coriolis / gravity columns. `dofRecordBase` is the record offset of the
// cohort's FIRST row: all its rows share a metatype, so they share the per-dof sign.
bool fetchArtiGeneralizedForceColumnOvStage(float* dst,
                                            const float* src,
                                            const ::ev4sio_physx::PxU32 count,
                                            const ::ev4sio_physx::PxU32 width,
                                            const ::ev4sio_physx::PxU32 rootDofs,
                                            const ::ev4sio_physx::PxU32 simMaxDofs,
                                            const ::ev4sio_physx::PxU32 dofRecordBase,
                                            const GpuArticulationDofRecord* dofRecords,
                                            cudaStream_t stream);

bool fetchArtiDofAttributeGravityAndCoriolis(float* dst,
                                             const float* src,
                                             const ::ev4sio_physx::PxU32 numDofs,
                                             const ::ev4sio_physx::PxU32 maxDofs,
                                             const ::ev4sio_physx::PxU32 simMaxDofs,
                                             const GpuArticulationDofRecord* dofRecords,
                                             const bool hasRootDofs);

bool fetchArtiCentroidalMomentumMatrices(float* dst,
                                         const float* src,
                                         const ::ev4sio_physx::PxU32 numElem,
                                         const ::ev4sio_physx::PxU32 maxDofs,
                                         const ::ev4sio_physx::PxU32 cenMomBlockSize,
                                         const ::ev4sio_physx::PxU32 simCenMomBlockSize,
                                         const ::ev4sio_physx::PxU32 startSimBiasForceBlock,
                                         const ::ev4sio_physx::PxU32 dofRecordBase,
                                         const GpuArticulationDofRecord* dofRecords,
                                         bool applyBodyOrderSign,
                                         cudaStream_t stream = nullptr);

bool fetchArtiJacobian(float* dst,
                       const float* src,
                       const ::ev4sio_physx::PxU32 numElements,
                       const ::ev4sio_physx::PxU32 jacobianSize,
                       const ::ev4sio_physx::PxU32 simJacobianSize,
                       const ::ev4sio_physx::PxU32 jacobianCols,
                       const ::ev4sio_physx::PxU32 rootDofs,
                       const ::ev4sio_physx::PxU32 dofRecordBase,
                       const GpuArticulationDofRecord* dofRecords,
                       bool applyBodyOrderSign,
                       cudaStream_t stream = nullptr);

bool fetchArtiDofAttribute(float* dst,
                           const float* src,
                           const ::ev4sio_physx::PxU32 numDofs,
                           const ::ev4sio_physx::PxU32 maxDofs,
                           const ::ev4sio_physx::PxU32 simMaxDofs,
                           const GpuArticulationDofRecord* dofRecords);

// ovstage joint-state gather:
//   dst[i] = dofScaleFor(recs[i], policy) * src[recs[i].viewArtiIdx * simMaxDofs + recs[i].physxDofIdx]
//
// `policy` selects which of the record's axis facts the attribute folds. An argument rather than a
// record field because one read shares the record list across every attribute.
bool fetchArtiDofAttributeOvStage(float* dst,
                                  const float* src,
                                  const ::ev4sio_physx::PxU32 numOutputs,
                                  const ::ev4sio_physx::PxU32 simMaxDofs,
                                  const DofScalePolicy policy,
                                  const ArticulationDofOvStageRecord* records,
                                  cudaStream_t stream);

// ovstage per-LINK gather for a fixed-width vector attribute:
//   dst[i*comp + c] = src[(recs[i].viewArtiIdx * maxLinks + recs[i].physxLinkIdx) * comp + c]
//
// `src` is a DENSE [count x maxLinks x comp] row from one of the link-parallel fetches, so `maxLinks`
// is the view's stride, not the scene's. No policy: the frame is resolved in the dense pass.
bool fetchArtiLinkVectorOvStage(float* dst,
                                const float* src,
                                const ::ev4sio_physx::PxU32 numOutputs,
                                const ::ev4sio_physx::PxU32 maxLinks,
                                const ::ev4sio_physx::PxU32 comp,
                                const ArticulationLinkOvStageRecord* records,
                                cudaStream_t stream);

// ovstage contact-sensor row compaction: dst[i*comp + c] = src[rowIdx[i]*comp + c]. A selected contact read
// reduces over the full sensor population and compacts its rows out of the result, so the selection never keys
// a reducer.
bool compactContactSensorRowsOvStage(float* dst,
                                     const float* src,
                                     const ::ev4sio_physx::PxU32* rowIdx,
                                     const ::ev4sio_physx::PxU32 numRows,
                                     const ::ev4sio_physx::PxU32 comp);

// ovstage SDF evaluation points: dst[i] = (src[i], 0), the caller's [N, Q, 3] points widened to the
// PxVec4 layout PxDirectGPUAPI::evaluateSDFDistances reads.
bool packSdfQueryPointsOvStage(::ev4sio_physx::PxVec4* dst, const ::ev4sio_physx::PxVec3* src, const ::ev4sio_physx::PxU32 count);

// ovstage raw-contact identities: dst[i] = pairs[2i + 1], the other body of each (sensor, other) pair the raw
// contact fill writes, as one contiguous column.
bool fetchRawContactOtherBodyOvStage(uint64_t* dst, const uint64_t* pairs, const ::ev4sio_physx::PxU32 count);

// ovstage rigid/link pose column, fused: PhysX-layout scratch -> the packed destination column in
// ONE pass. Resolves rigid-dynamic vs articulation-link source (de-padding the [numArti x
// simMaxLinks] link block), subtracts the subspace origin, and writes only the requested component
// -- position (3 floats) or orientation (4) -- so no [N,7] intermediate is materialized.
//
// `outRecordIdx` gives output slot i its record: null means identity (slot i -> record i), a list
// means an arbitrary subset/reorder of a larger view. `numOutputs` is the destination row count.
//
// The sources hold the whole view by default. A packed fetch (ADR-0055) holds only the read's rows: `packedRd` puts
// output slot i's rigid dynamic at source slot i, and `artiSlot[i]` names the fetched slot of output i's link's
// articulation. A row whose rigid dynamic or articulation lies past the `numFetched` the sources hold is zeroed.
bool fetchRbPoseColumnOvStage(float* dst,
                              const ::ev4sio_physx::PxTransform* actorData,
                              const ::ev4sio_physx::PxTransform* linkTransforms,
                              const GpuRigidBodyRecord* records,
                              const ::ev4sio_physx::PxU32* outRecordIdx,
                              const ::ev4sio_physx::PxU32 numOutputs,
                              const ::ev4sio_physx::PxU32 simMaxLinks,
                              const bool wantOrientation,
                              cudaStream_t stream,
                              const bool packedRd = false,
                              const ::ev4sio_physx::PxU32* artiSlot = nullptr,
                              const ::ev4sio_physx::PxU32 numFetched = 0xffffffffu);

// ovstage rigid/link velocity column, fused. Same record/indirection contract; the caller passes
// the linear or angular source pair, so the component selector is the pointers themselves.
bool fetchRbVelocityColumnOvStage(float* dst,
                                  const ::ev4sio_physx::PxVec3* rdData,
                                  const ::ev4sio_physx::PxVec3* linkData,
                                  const GpuRigidBodyRecord* records,
                                  const ::ev4sio_physx::PxU32* outRecordIdx,
                                  const ::ev4sio_physx::PxU32 numOutputs,
                                  const ::ev4sio_physx::PxU32 simMaxLinks,
                                  cudaStream_t stream,
                                  const bool packedRd = false,
                                  const ::ev4sio_physx::PxU32* artiSlot = nullptr,
                                  const ::ev4sio_physx::PxU32 numFetched = 0xffffffffu);

// ovstage rigid/link world-scale column: a plain gather from the view's per-record scale copy. Same
// `outRecordIdx` contract as the pose and velocity columns; there is no engine source to resolve.
bool fetchRbWorldScaleColumnOvStage(float* dst,
                                    const ::ev4sio_physx::PxVec3* scales,
                                    const ::ev4sio_physx::PxU32* outRecordIdx,
                                    const ::ev4sio_physx::PxU32 numOutputs,
                                    cudaStream_t stream);

// ovstage joint-DOF write (ADR-0012): the exact inverse of fetchArtiDofAttributeOvStage.
//
// THE WRITE MULTIPLIES BY THE RECORD'S INVERSE SCALE (dofInverseScaleFor), and that is the whole risk in
// this function. The read multiplies by dofScaleFor, which folds the unit convention (rad->deg on angular
// axes) and the sign; the write has to undo both. It uses the record's stored deg->rad constant rather than
// dividing by the rad->deg one, because the float reciprocal of one is not the other. Applying the read's
// factor here instead would reproduce the 57.295x joint-velocity error the unit-conversion contract exists
// to prevent (ADR-0001 section 7.5): descriptors carry engine-native units, and the boundary converts --
// once, in each direction.
//
// Writes into the scene's DOF scratch in PhysX layout, which the caller has already filled with the
// articulations' current values. That read-modify-write is not optional: PhysX writes whole
// articulation DOF blocks, so a DOF this call does not touch has to already hold its current value
// or it would be zeroed.
bool submitArtiDofAttributeOvStage(float* dofScalars,
                                   const float* src,
                                   const ::ev4sio_physx::PxU32 numOutputs,
                                   const ::ev4sio_physx::PxU32 simMaxDofs,
                                   const ArticulationDofOvStageRecord* records,
                                   // Which folds the scatter undoes. Not derivable from the write
                                   // flag: the four state/target columns are eAngularSigned and the
                                   // actuation force is eSigned, yet all five are JOINT_* writes.
                                   DofScalePolicy policy,
                                   cudaStream_t stream = nullptr,
                                   // A masked commit's selection (ADR-0052), one entry per output.
                                   const uint8_t* mask = nullptr);

// ovstage rigid-body write, PACKED form (ADR-0012). The pair below replaces a flags-and-compaction
// scatter, and the reason is architectural rather than performance: fillRdTransforms computes its
// element count on the HOST, from a thrust::copy_if iterator difference, so any path built on it
// must block. The read carries no host block at all (ADR-0008 W2 / Decision 7), and a write that
// blocks cannot join that contract. Building the packed pair directly makes the count numOutputs --
// known on the host before the kernel launches -- so nothing has to be read back.
//
// It also takes the write OUT of the shared scene buffers: nothing here touches mRdPoseDev or the
// velocity scratch, so a read gather and a write scatter over one scene no longer alias, and the
// SharedDeviceBuffer ordering that would otherwise be needed between them does not arise.
//
// PhysX pairs the two arrays by position: the data for the body whose GPU index sits at slot x must
// sit at slot x. Both are therefore built from the same row list in one pass.

// Packed GPU-index list for the rows this write covers: dstIdx[i] is the DirectGPU index of the
// body at output slot i.
//
// A record with no GPU row (tensorRdIdx == sentinel: a disabled body, or an articulation link) has
// no index to write and would poison the whole call, so the CALLER must ensure the row list carries
// none. That is guaranteed by construction -- the ovstage rigid enumeration yields only rigid
// dynamics, and refreshRdGpuIndices fails the op if any lacks a GPU row -- and is checked on the
// host where the list is built, because a kernel cannot report it without a synchronize.
bool submitRbPackedIndicesOvStage(::ev4sio_physx::PxRigidDynamicGPUIndex* dstIdx,
                                 const ::ev4sio_physx::PxRigidDynamicGPUIndex* rdGpuIndices,
                                 const GpuRigidBodyRecord* records,
                                 const ::ev4sio_physx::PxU32* outRecordIdx,
                                 const ::ev4sio_physx::PxU32 numOutputs,
                                 cudaStream_t stream = nullptr);

// Overlay one pose component onto a packed transform block the caller has already filled with the
// bodies' current poses. The half not being written survives, which is the whole point: PhysX has a
// single eGLOBAL_POSE write and a session carries one attribute.
//
// The subspace origin is ADDED here where fetchRbPoseColumnOvStage subtracts it, so a column read,
// edited and written back round-trips to the value it started from.
bool submitRbPackedPoseOvStage(::ev4sio_physx::PxTransform* packedPose,
                                const float* src,
                                const GpuRigidBodyRecord* records,
                                const ::ev4sio_physx::PxU32* outRecordIdx,
                                const ::ev4sio_physx::PxU32 numOutputs,
                                const bool wantOrientation,
                                cudaStream_t stream = nullptr,
                                const uint8_t* mask = nullptr);

// A masked rigid velocity, force or torque commit (ADR-0052): overlays the selected rows of a dense
// [N,3] column onto `dst`. An unselected row keeps `dst`, which the caller pre-read with the current
// value, or becomes zero when `zeroUnselected` (a force or torque, which PhysX cannot read back).
bool overlayVec3ColumnOvStage(::ev4sio_physx::PxVec3* dst,
                              const float* src,
                              const uint8_t* mask,
                              ::ev4sio_physx::PxU32 numOutputs,
                              bool zeroUnselected,
                              cudaStream_t stream);

// ovstage LINK force/wrench write (ADR-0012). The articulation counterpart of
// submitRbWrenchOvStage: PhysX addresses a link's force as (articulation, link) rather than as a
// rigid-dynamic row, so the destination slot is `tensorArtiIdx * simMaxLinks + linkIdx` and the data
// block covers the WHOLE view.
//
// `comps` is 3 for a plain force and 9 for a wrench; the torque-about-COM conversion runs only in the
// 9 case, using the link's own pose and centre of mass.
bool submitLinkWrenchOvStage(::ev4sio_physx::PxVec3* linkForces,
                              ::ev4sio_physx::PxVec3* linkTorques,
                              const float* src,
                              const ::ev4sio_physx::PxTransform* linkTransforms,
                              const ::ev4sio_physx::PxVec3* comsByRecord,
                              const GpuRigidBodyRecord* records,
                              const ::ev4sio_physx::PxU32* rows,
                              const ::ev4sio_physx::PxU32* rowCompactArti,
                              ::ev4sio_physx::PxU32 numOutputs,
                              ::ev4sio_physx::PxU32 simMaxLinks,
                              ::ev4sio_physx::PxU32 comps,
                              cudaStream_t stream = nullptr,
                              const uint8_t* mask = nullptr);

// ovstage WRENCH write (ADR-0012). Turns one [N,9] column -- force(3), torque(3), world
// application point(3) -- into the two dense PxVec3 arrays PhysX's eFORCE / eTORQUE write types take.
//
// The conversion is the whole reason `wrench` is not just `force`: a load applied AWAY from the
// centre of mass also produces a torque about it. Same expression the tensor binding's applyForces
// helper uses, so the two APIs agree:
//
//     comWorld = bodyPose.transform(comLocal)
//     torque   = srcTorque + (point - comWorld) x force
//
// `poses` and `coms` are indexed by OUTPUT slot i (poses were read packed through the same index
// list), while `coms` is indexed by the body's record row -- hence `rows`.
bool submitRbWrenchOvStage(::ev4sio_physx::PxVec3* outForces,
                            ::ev4sio_physx::PxVec3* outTorques,
                            const float* src,
                            const ::ev4sio_physx::PxTransform* poses,
                            const ::ev4sio_physx::PxVec3* comsByRecord,
                            const ::ev4sio_physx::PxU32* rows,
                            ::ev4sio_physx::PxU32 numOutputs,
                            cudaStream_t stream = nullptr,
                            const uint8_t* mask = nullptr);

// ovstage articulation-ROOT write (ADR-0012). Overlay one component onto the view-wide root block
// the caller has already filled with the articulations' current root state, indexing it through the
// ovstage row list: output i addresses view articulation `rows[i]`.
//
// Read-modify-write is not optional here for the same reason it is not for DOFs, but for a second
// reason on top: PhysX has ONE eROOT_GLOBAL_POSE write covering position and orientation together,
// and a session carries one attribute -- so the half not being written has to already be in the
// block. And the write covers the WHOLE view (a subset write would need a compacted index list whose
// length comes off the host), so every articulation the query did not match has to hold its current
// value too.
//
// The subspace origin is ADDED here where fetchArtiRootTransforms subtracts it, so a root pose read,
// edited and written back round-trips to the value it started from.
bool submitArtiRootPoseOvStage(::ev4sio_physx::PxTransform* rootBlock,
                                const float* src,
                                const ::ev4sio_physx::PxU32* rows,
                                const GpuArticulationRootRecord* rootRecords,
                                ::ev4sio_physx::PxU32 numOutputs,
                                bool wantOrientation,
                                cudaStream_t stream = nullptr,
                                const uint8_t* mask = nullptr);

// The velocity counterpart. No origin: a velocity is frame-invariant under a translation, which is
// why the root velocity read applies none either. Linear and angular are separate PhysX write types,
// so each has its own block and neither has to preserve the other.
bool submitArtiRootVelocityOvStage(::ev4sio_physx::PxVec3* velBlock,
                                    const float* src,
                                    const ::ev4sio_physx::PxU32* rows,
                                    ::ev4sio_physx::PxU32 numOutputs,
                                    cudaStream_t stream = nullptr,
                                    const uint8_t* mask = nullptr);

// ovstage tendon-property WRITE (ADR-0012): the inverse of fetchArtiTendonPropertyOvStage.
//
// NO SCALE, and that is a decision rather than an omission. A fixed tendon's length is a weighted
// sum of joint positions, so its rest length, limits and offset are in generalized-coordinate units
// and its stiffness is force per those units -- the unit question does arise here. It is already
// answered upstream: updateTendonAxisSingleGearing folds rad2deg into the gearing coefficient, and
// everything after that is passthrough. The READ applies no scale for that reason, so the write
// applies none either; converting here would break the round trip.
//
// Writes into the scene's tendon block in PhysX layout, which the caller has already filled with the
// articulations' current tendon structs. That read-modify-write is not optional: PhysX writes whole
// tendon structs, so a property this call does not address must already hold its current value.
//
// `mode` other than eSet is a one-sided limit write (TendonFieldWrite): `fieldOffset` is the (low, high) pair and
// `src` carries one value per row.
bool submitArtiTendonPropertyOvStage(float* tendonStructs,
                                     const float* src,
                                     ::ev4sio_physx::PxU32 numOutputs,
                                     ::ev4sio_physx::PxU32 simMaxTendons,
                                     ::ev4sio_physx::PxU32 structFloats,
                                     ::ev4sio_physx::PxU32 fieldOffset,
                                     ::ev4sio_physx::PxU32 comp,
                                     const ArticulationTendonOvStageRecord* records,
                                     cudaStream_t stream = nullptr,
                                     const uint8_t* mask = nullptr,
                                     TendonFieldWrite mode = TendonFieldWrite::eSet);


bool submitArtiDofAttribute(float* dst,
                            const float* src,
                            const ::ev4sio_physx::PxU32* srcArtiIndices,
                            ::ev4sio_physx::PxU32* dirtyArtiGpuIndices,
                            const ::ev4sio_physx::PxU32 numDofs,
                            const ::ev4sio_physx::PxU32 maxDofs,
                            const ::ev4sio_physx::PxU32 simMaxDofs,
                            const ::ev4sio_physx::PxU32 maxArtis,
                            const GpuArticulationDofRecord* dofRecords);

// ovstage tendon gather: one kernel for every tendon attribute, on both tendon kinds.
//
// PxGpuSpatialTendonData and PxGpuFixedTendonData are plain float structs, so an attribute is just
// a float offset and a component count into a fixed stride -- there is nothing per-attribute left
// for a kernel of its own to do. `structFloats` is 4 for spatial tendons and 8 for fixed ones.
//
//   dst[i*comp + c] = src[(recs[i].viewArtiIdx * simMaxTendons + recs[i].tendonIdx) * structFloats
//                         + fieldOffset + c]
bool fetchArtiTendonPropertyOvStage(float* dst,
                                    const float* src,
                                    const ::ev4sio_physx::PxU32 numOutputs,
                                    const ::ev4sio_physx::PxU32 simMaxTendons,
                                    const ::ev4sio_physx::PxU32 structFloats,
                                    const ::ev4sio_physx::PxU32 fieldOffset,
                                    const ::ev4sio_physx::PxU32 comp,
                                    const ArticulationTendonOvStageRecord* records,
                                    cudaStream_t stream);

// ovstage point-instancer columns: instancer-local pose (reframed on the device) and world-frame
// velocity, both scattered into per-instancer destination sub-ranges named by `offsets`.
bool fetchInstancerPoseColumnOvStage(float* dst,
                                     const ::ev4sio_physx::PxTransform* actorData,
                                     const GpuRigidBodyRecord* rbRecords,
                                     const GpuPointInstancerRecord* records,
                                     const InstancerAffine* instancerInverses,
                                     const ::ev4sio_physx::PxU32* offsets,
                                     ::ev4sio_physx::PxU32 numInstances,
                                     bool wantOrientation,
                                     cudaStream_t stream);

bool fetchInstancerVelocityColumnOvStage(float* dst,
                                         const ::ev4sio_physx::PxVec3* rdData,
                                         const GpuRigidBodyRecord* rbRecords,
                                         const GpuPointInstancerRecord* records,
                                         const ::ev4sio_physx::PxU32* offsets,
                                         ::ev4sio_physx::PxU32 numInstances,
                                         cudaStream_t stream);

// The ovstage point-instancer WRITE (ADR-0012). One instancer per call, because the write
// hands each instancer's array to the caller as its own group and commits them independently.
//
// A pose write is a READ-MODIFY-WRITE here, exactly as it is on the host path and for the same
// reason: a session carries one attribute, but a pose needs both halves, so the half the column does
// not supply is recovered from the body's current world pose (reframed to local), substituted, and
// composed back out. That is why this takes the forward AND inverse transforms.
//
// `records` points at THIS instancer's range and `numOutputs` is its length, so output slot i is
// simply thread i -- the view groups its records by instancer, which removes both a per-instance
// filter and a per-commit slot-map upload. Holes never appear in a range: an instance with no live
// body is dropped when the records are built.
bool submitInstancerPoseColumnOvStage(::ev4sio_physx::PxTransform* packedPose,
                                      const float* src,
                                      const ::ev4sio_physx::PxTransform* actorData,
                                      const GpuRigidBodyRecord* rbRecords,
                                      const GpuPointInstancerRecord* records,
                                      const InstancerAffine* instancerInverses,
                                      const InstancerAffine* instancerForwards,
                                      ::ev4sio_physx::PxU32 numOutputs,
                                      ::ev4sio_physx::PxU32 instancerIdx,
                                      bool wantOrientation,
                                      cudaStream_t stream = nullptr,
                                      // A masked commit's selection, indexed by instance slot.
                                      const uint8_t* mask = nullptr,
                                      // A session commit's pre-read of exactly these instances' current
                                      // poses, by output slot; null reads `actorData` by rigid row.
                                      const ::ev4sio_physx::PxTransform* currentByOutput = nullptr);

// The velocity counterpart. No reframe in either direction -- a velocity is published in world space,
// so the instancer transform does not apply to it.
bool submitInstancerVelocityColumnOvStage(::ev4sio_physx::PxVec3* packedVel,
                                          const float* src,
                                          const GpuPointInstancerRecord* records,
                                          ::ev4sio_physx::PxU32 numOutputs,
                                          cudaStream_t stream = nullptr,
                                          const uint8_t* mask = nullptr);

bool fetchFixedTendonStiffness(float* dst,
                               const ::ev4sio_physx::PxGpuFixedTendonData* tendonProperties,
                               const ::ev4sio_physx::PxU32 numTendons,
                               const ::ev4sio_physx::PxU32 maxTendons,
                               const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchFixedTendonDamping(float* dst,
                             const ::ev4sio_physx::PxGpuFixedTendonData* tendonProperties,
                             const ::ev4sio_physx::PxU32 numTendons,
                             const ::ev4sio_physx::PxU32 maxTendons,
                             const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchFixedTendonLimitStiffness(float* dst,
                                    const ::ev4sio_physx::PxGpuFixedTendonData* tendonProperties,
                                    const ::ev4sio_physx::PxU32 numTendons,
                                    const ::ev4sio_physx::PxU32 maxTendons,
                                    const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchFixedTendonLimits(float* dst,
                            const ::ev4sio_physx::PxGpuFixedTendonData* tendonProperties,
                            const ::ev4sio_physx::PxU32 numTendons,
                            const ::ev4sio_physx::PxU32 maxTendons,
                            const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchFixedTendonRestLength(float* dst,
                                const ::ev4sio_physx::PxGpuFixedTendonData* tendonProperties,
                                const ::ev4sio_physx::PxU32 numTendons,
                                const ::ev4sio_physx::PxU32 maxTendons,
                                const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchFixedTendonOffset(float* dst,
                            const ::ev4sio_physx::PxGpuFixedTendonData* tendonProperties,
                            const ::ev4sio_physx::PxU32 numTendons,
                            const ::ev4sio_physx::PxU32 maxTendons,
                            const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchSpatialTendonStiffness(float* dst,
                                 const ::ev4sio_physx::PxGpuSpatialTendonData* tendonProperties,
                                 const ::ev4sio_physx::PxU32 numTendons,
                                 const ::ev4sio_physx::PxU32 maxTendons,
                                 const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchSpatialTendonDamping(float* dst,
                               const ::ev4sio_physx::PxGpuSpatialTendonData* tendonProperties,
                               const ::ev4sio_physx::PxU32 numTendons,
                               const ::ev4sio_physx::PxU32 maxTendons,
                               const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchSpatialTendonLimitStiffness(float* dst,
                                      const ::ev4sio_physx::PxGpuSpatialTendonData* tendonProperties,
                                      const ::ev4sio_physx::PxU32 numTendons,
                                      const ::ev4sio_physx::PxU32 maxTendons,
                                      const ::ev4sio_physx::PxU32 simMaxTendons);

bool fetchSpatialTendonOffset(float* dst,
                              const ::ev4sio_physx::PxGpuSpatialTendonData* tendonProperties,
                              const ::ev4sio_physx::PxU32 numTendons,
                              const ::ev4sio_physx::PxU32 maxTendons,
                              const ::ev4sio_physx::PxU32 simMaxTendons);

bool submitArtiFixedTendonProperties(::ev4sio_physx::PxGpuFixedTendonData* dst,
                                     const float* stiffnesses,
                                     const float* dampings,
                                     const float* limitStiffnesses,
                                     const float* limits,
                                     const float* restLengths,
                                     const float* offsets,
                                     const ::ev4sio_physx::PxU32* srcIndices,
                                     ::ev4sio_physx::PxU32* dirtyArtiGpuIndices,
                                     const ::ev4sio_physx::PxU32 numIndices,
                                     const ::ev4sio_physx::PxU32 maxTendons,
                                     const ::ev4sio_physx::PxU32 simMaxTendons,
                                     const GpuArticulationFixedTendonRecord* tensorRecords);

bool submitArtiSpatialTendonProperties(::ev4sio_physx::PxGpuSpatialTendonData* dst,
                                       const float* stiffnesses,
                                       const float* dampings,
                                       const float* limitStiffnesses,
                                       const float* offsets,
                                       const ::ev4sio_physx::PxU32* srcIndices,
                                       ::ev4sio_physx::PxU32* dirtyArtiGpuIndices,
                                        const ::ev4sio_physx::PxU32 numIndices,
                                       const ::ev4sio_physx::PxU32 maxTendons,
                                       const ::ev4sio_physx::PxU32 simMaxTendons,
                                       const GpuArticulationSpatialTendonRecord* tensorRecords);

bool fetchArtiLinkIncomingJointForce(PhysxGpuSpatialForces* dst,
                                     const PhysxGpuSpatialForces* src,
                                     const ::ev4sio_physx::PxTransform* linkTransforms,
                                     const ::ev4sio_physx::PxU32 numLinks,
                                     const ::ev4sio_physx::PxU32 maxLinks,
                                     const ::ev4sio_physx::PxU32 simMaxLinks,
                                     const GpuArticulationLinkRecord* linkRecords,
                                     cudaStream_t stream = nullptr);

bool fetchDofProjectionForce(float* dst,
                             const PhysxGpuSpatialForces* src,
                             const ::ev4sio_physx::PxTransform* linkTransforms,
                             const ::ev4sio_physx::PxU32 numLinks,
                             const ::ev4sio_physx::PxU32 maxLinks,
                             const ::ev4sio_physx::PxU32 simMaxLinks,
                             const GpuArticulationLinkRecord* linkRecords,
                             cudaStream_t stream = nullptr);

bool submitArtiLinkForces(::ev4sio_physx::PxVec3* dstLinkForces,
                          ::ev4sio_physx::PxVec3* dstLinkTorques,
                          ::ev4sio_physx::PxU32* dirtyArtiGpuIndices,
                          const ::ev4sio_physx::PxTransform* linkTransforms,
                          const ::ev4sio_physx::PxVec3* actorAndLinksComs,
                          const ::ev4sio_physx::PxVec3* srcForces,
                          const ::ev4sio_physx::PxVec3* srcTorques,
                          const ::ev4sio_physx::PxVec3* srcPositions,
                          const ::ev4sio_physx::PxU32* srcArtiIndices,
                          const ::ev4sio_physx::PxU32 numArtIndices,
                          const ::ev4sio_physx::PxU32 numLinks,
                          const ::ev4sio_physx::PxU32 simMaxLinks,
                          const GpuArticulationLinkRecord* linkRecords,
                          const bool isGlobal,
                          const bool submitForces,
                          const bool submitTorques,
                          const bool applyAtPosition);

//
// rigid bodies
//
::ev4sio_physx::PxU32 fillRdTransforms(SingleAllocPolicy& policy,
                                ::ev4sio_physx::PxTransform* rdTransformDev,
                                ::ev4sio_physx::PxU32* indicesRet,
                                const ::ev4sio_physx::PxU32* allRdIndices,
                                const ActorGpuFlags* rdDirtyFlags,
                                const ActorGpuFlag::Enum flag,
                                const ::ev4sio_physx::PxU32 numRds);
::ev4sio_physx::PxU32 fillRdVelocities(SingleAllocPolicy& policy,
                                ::ev4sio_physx::PxVec3* rdLinVelDev,
                                ::ev4sio_physx::PxVec3* rdAngVelDev,
                                ::ev4sio_physx::PxU32* indicesRet,
                                const ::ev4sio_physx::PxU32* allRdIndices,
                                const ActorGpuFlags* rdDirtyFlags,
                                const ActorGpuFlag::Enum flag,
                                const ::ev4sio_physx::PxU32 numRds);

::ev4sio_physx::PxU32 fillRdFT(SingleAllocPolicy& policy,
                        ::ev4sio_physx::PxVec3* FT,
                        ::ev4sio_physx::PxU32* indicesRet,
                        const ::ev4sio_physx::PxU32* allRdIndices,
                        const ActorGpuFlags* rdDirtyFlags,
                        const ActorGpuFlag::Enum flag,
                        const ::ev4sio_physx::PxU32 numRds);

void exclusiveScan(::ev4sio_physx::PxU32* dstCounts, ::ev4sio_physx::PxU32* dstStartIndices, const ::ev4sio_physx::PxU32 numElem);

bool fetchRbTransforms(TensorTransform* dst,
                       const ::ev4sio_physx::PxTransform* actorData,
                       const ::ev4sio_physx::PxTransform* linkTransforms,
                       const ::ev4sio_physx::PxU32 numBodies,
                       const ::ev4sio_physx::PxU32 simMaxLinks,
                       const GpuRigidBodyRecord* rbRecords);

bool submitRbTransforms(::ev4sio_physx::PxTransform* dstActorData,
                        ::ev4sio_physx::PxTransform* dstRootTransforms,
                        ActorGpuFlags* rdDirtyFlags,
                        ArticulationGpuFlags* artiDirtyFlags,
                        const TensorTransform* src, // transforms in RigidBodyView
                        const ::ev4sio_physx::PxU32* srcRbIndices, // indices in RigidBodyView
                        const ::ev4sio_physx::PxU32 numRbIndices,
                        const ::ev4sio_physx::PxU32 numBodies,
                        const GpuRigidBodyRecord* rbRecords);


bool fetchRbVelAcc(TensorVelAcc* dst,
                   const ::ev4sio_physx::PxVec3* actorLinVelAcc,
                   const ::ev4sio_physx::PxVec3* actorAngVelAcc,
                   const ::ev4sio_physx::PxVec3* linkLinVelAcc,
                   const ::ev4sio_physx::PxVec3* linkAngVelAcc,
                   const ::ev4sio_physx::PxU32 numBodies,
                   const ::ev4sio_physx::PxU32 maxArtLinks,
                   const GpuRigidBodyRecord* rbRecords);

bool submitRbVelocities(::ev4sio_physx::PxVec3* actorLinVel,
                        ::ev4sio_physx::PxVec3* actorAngVel,
                        ::ev4sio_physx::PxVec3* linkLinVel,
                        ::ev4sio_physx::PxVec3* linkAngVel,
                        ActorGpuFlags* rdDirtyFlags,
                        ArticulationGpuFlags* artiDirtyFlags,
                        const TensorVelAcc* src, // velocities in RigidBodyView
                        const ::ev4sio_physx::PxU32* srcRbIndices, // indices in RigidBodyView
                        const ::ev4sio_physx::PxU32 numRbIndices,
                        const ::ev4sio_physx::PxU32 numRbs,
                        const GpuRigidBodyRecord* rbRecords);


bool submitRbForces(::ev4sio_physx::PxVec3* dstActorForces, // forces in physx rd staging buffer
                    ::ev4sio_physx::PxVec3* dstActorTorques, // torques in physx rd staging buffer
                    ::ev4sio_physx::PxVec3* dstLinkForces, // forces in physx link staging buffer
                    ::ev4sio_physx::PxVec3* dstLinkTorques, // forces in physx link staging buffer
                    ActorGpuFlags* rdDirtyFlags,
                    ArticulationGpuFlags* artiDirtyFlags,
                    ArticulationGpuFlags* artiLinksDirtyFlags,
                    const ::ev4sio_physx::PxTransform* linkTransforms,
                    const ::ev4sio_physx::PxTransform* actorData,
                    const ::ev4sio_physx::PxVec3* actorAndLinksComs,
                    const ::ev4sio_physx::PxVec3* srcForces, // forces in RigidBodyView
                    const ::ev4sio_physx::PxVec3* srcTorques, // torques in RigidBodyView
                    const ::ev4sio_physx::PxVec3* srcPositions, // positions to apply forces in RigidBodyView
                    const ::ev4sio_physx::PxU32* srcRbIndices, // indices in RigidBodyView
                    const ::ev4sio_physx::PxU32 numRbIndices,
                    const ::ev4sio_physx::PxU32 simMaxLinks,
                    const GpuRigidBodyRecord* rbRecords,
                    const bool isGlobal,
                    const bool submitForces,
                    const bool submitTorques,
                    const bool applyAtPosition);

// Rigid contacts
//

bool fetchNetRigidContactForces(::ev4sio_physx::PxVec3* netForces,
                                const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                                ::ev4sio_physx::PxU32 numContactPairs,
                                const RigidContactReferentTables& referents,
                                float timeStepInv,
                                ForceComponent component);

bool fetchRigidContactForceMatrix(::ev4sio_physx::PxVec3* forceMatrix,
                                  const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                                  ::ev4sio_physx::PxU32 numContactPairs,
                                  ::ev4sio_physx::PxU32 numFilters,
                                  const RigidContactReferentTables& referents,
                                  float timeStepInv,
                                  const GpuRigidContactFilterIdPair* filterLookup,
                                  ForceComponent component);

bool fetchRigidContactCount(::ev4sio_physx::PxU32* countMatrix,
                            const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                            ::ev4sio_physx::PxU32 numContactPairs,
                            ::ev4sio_physx::PxU32 numFilters,
                            const RigidContactReferentTables& referents,
                            const GpuRigidContactFilterIdPair* filterLookup);

bool fetchRigidContactData(::ev4sio_physx::PxReal* forceBuffer,
                           ::ev4sio_physx::PxVec3* pointBuffer,
                           ::ev4sio_physx::PxVec3* normalBuffer,
                           ::ev4sio_physx::PxReal* separationBuffer,
                           ::ev4sio_physx::PxU32* countMatrix,
                           ::ev4sio_physx::PxU32* startIndicesMatrix,
                           const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                           ::ev4sio_physx::PxU32 numContactPairs,
                           ::ev4sio_physx::PxU32 numFilters,
                           ::ev4sio_physx::PxU32 numDataPoints,
                           const RigidContactReferentTables& referents,
                           float timeStepInv,
                           const GpuRigidContactFilterIdPair* filterLookup);

bool fetchRigidFrictionData(::ev4sio_physx::PxVec3* forceBuffer,
                            ::ev4sio_physx::PxVec3* pointBuffer,
                            ::ev4sio_physx::PxU32* countMatrix,
                            ::ev4sio_physx::PxU32* startIndicesMatrix,
                            const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                            ::ev4sio_physx::PxU32 numContactPairs,
                            ::ev4sio_physx::PxU32 numFilters,
                            ::ev4sio_physx::PxU32 numDataPoints,
                            const RigidContactReferentTables& referents,
                            float timeStepInv,
                            const GpuRigidContactFilterIdPair* filterLookup);

bool fetchFrictionCount(::ev4sio_physx::PxU32* countMatrix,
                        const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                        ::ev4sio_physx::PxU32 numContactPairs,
                        ::ev4sio_physx::PxU32 numFilters,
                        const RigidContactReferentTables& referents,
                        const GpuRigidContactFilterIdPair* filterLookup);

// Raw contact data (no filter matching - returns all contacts for sensors)
bool fetchRawRigidContactCount(::ev4sio_physx::PxU32* countBuffer,
                               const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                               ::ev4sio_physx::PxU32 numContactPairs,
                               const RigidContactReferentTables& referents);

bool fetchRawRigidContactData(::ev4sio_physx::PxReal* forceBuffer,
                              ::ev4sio_physx::PxVec3* pointBuffer,
                              ::ev4sio_physx::PxVec3* normalBuffer,
                              ::ev4sio_physx::PxReal* separationBuffer,
                              uint64_t* actorIdBuffer,
                              ::ev4sio_physx::PxU32* countBuffer,
                              ::ev4sio_physx::PxU32* startIndicesBuffer,
                              const ::ev4sio_physx::PxGpuContactPair* contactPairs,
                              ::ev4sio_physx::PxU32 numContactPairs,
                              ::ev4sio_physx::PxU32 numDataPoints,
                              const RigidContactReferentTables& referents,
                              float timeStepInv,
                              const GpuActorPathIdPair* actorPathLookup,
                              ::ev4sio_physx::PxU32 numActorPathPairs);

// Clamp per-sensor counts and start indices so start[i] + count[i] <= cap.
// Call after fetchRawRigidContactData to correct the
// atomically-incremented counts that may exceed the written prefix.
bool clampContactLayout(::ev4sio_physx::PxU32* counts, ::ev4sio_physx::PxU32* startIndices,
                        ::ev4sio_physx::PxU32 numSensors, ::ev4sio_physx::PxU32 cap);

// Interleave contiguous per-sensor counts/start indices into a caller-facing
// (numSensors, 2) layout tensor: column 0 count, column 1 start index.
bool packSensorLayout(::ev4sio_physx::PxU32* dstLayout, const ::ev4sio_physx::PxU32* counts,
                      const ::ev4sio_physx::PxU32* startIndices, ::ev4sio_physx::PxU32 numSensors);

bool submitSdfQueryPoints(::ev4sio_physx::PxVec4* dst,
                          const ::ev4sio_physx::PxVec3* src,
                          const GpuSdfShapeRecord* sdfRecords,
                          const ::ev4sio_physx::PxU32 numIndices,
                          const ::ev4sio_physx::PxU32 maxPointsPerShape);

// deformable bodies

bool fetchDeformableBodyUInt4Data(::ev4sio_physx::PxU32* dst,
                                  const GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::ev4sio_physx::PxU32 numBodies,
                                  const ::ev4sio_physx::PxU32 dstMaxElementsPerBody);

bool fetchDeformableBodyUInt3Data(::ev4sio_physx::PxU32* dst,
                                  const GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::ev4sio_physx::PxU32 numBodies,
                                  const ::ev4sio_physx::PxU32 dstMaxElementsPerBody);

bool fetchDeformableBodyVec3Data(::ev4sio_physx::PxVec3* dst,
                                 const GpuDeformableBodyRecord* bodyRecords,
                                 const DeformableBodyData::Enum dataFlag,
                                 const ::ev4sio_physx::PxU32 numBodies,
                                 const ::ev4sio_physx::PxU32 dstMaxElementsPerBody);

bool fetchDeformableBodyVec4Data(::ev4sio_physx::PxVec4* dst,
                                 const GpuDeformableBodyRecord* bodyRecords,
                                 const DeformableBodyData::Enum dataFlag,
                                 const ::ev4sio_physx::PxU32 numBodies,
                                 const ::ev4sio_physx::PxU32 dstMaxElementsPerBody);

bool submitDeformableBodyVec3Data(const ::ev4sio_physx::PxVec3* src,
                                  const ::ev4sio_physx::PxU32* indices,
                                  GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::ev4sio_physx::PxU32 numBodies,
                                  const ::ev4sio_physx::PxU32 srcMaxElementsPerBody);

bool submitDeformableBodyVec4Data(const ::ev4sio_physx::PxVec4* src,
                                  const ::ev4sio_physx::PxU32* indices,
                                  GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const ::ev4sio_physx::PxU32 numBodies,
                                  const ::ev4sio_physx::PxU32 srcMaxElementsPerBody);


//
// mask -> indices compaction
//
// Converts a uint8 mask [N] into a compact indices array [K] where K = number of nonzero mask elements.
// Uses Thrust's copy_if for GPU-friendly parallel stream compaction.
// This runs on the CUDA stream associated with the provided execution policy (currently the default stream).
// Returns true on success and writes K (number of selected indices) to outK.
bool compactMaskToIndices(SingleAllocPolicy& policy,
                          ::ev4sio_physx::PxU32* indicesOut,
                          const uint8_t* maskDev,
                          ::ev4sio_physx::PxU32 N,
                          ::ev4sio_physx::PxU32& outK);

// ovstage deformable sim-mesh columns. ONE launch covers every body in the read: each thread owns a
// vertex, finds its body through `records`, and writes straight into that body's slice of the
// destination column. `transforms` is the per-body world-to-sim-mesh matrix, applied only when
// `reframe` is set: points are published sim-mesh-local, velocities in world frame.
//
// The source is PxVec4, xyz plus inverse mass, for PhysX sim-mesh and particle buffers, or PxVec3 for the
// runtime's skinned-vertex buffer. The destination is a packed PxVec3 column, so the compaction happens
// here. `layout` selects the source element for the whole launch, since every set of a view shares one.
bool fetchPointSetColumnOvStage(float* dst,
                                const GpuPointSetReadRecord* records,
                                const PointSetTransform* transforms,
                                ::ev4sio_physx::PxU32 numSets,
                                ::ev4sio_physx::PxU32 maxPointsPerSet,
                                bool reframe,
                                PointSetSourceLayout layout,
                                cudaStream_t stream);

// The ovstage POINT-SET write (ADR-0012): scatter one body's caller column into PhysX's
// own device buffer. The inverse of fetchPointSetColumnOvStage, but per BODY rather than rectangular
// over (set, point): a write session commits one group at a time, so there is no ragged set of them
// to launch across and no idle threads to trade away.
//
// `dst` is PhysX's buffer, whose fourth lane is INVERSE MASS on a position column. It is read back
// and re-stored per element rather than written: the caller's column is vec3, and a four-lane store
// would set every vertex's mass to infinite -- which the read cannot show, because it drops that
// lane, and which surfaces only as a body that has quietly stopped responding to force.
//
// `reframe` selects what the column MEANS, mirroring the gather: points arrive sim-mesh-local and
// are carried to world by `localToWorld`, velocities are world already and are stored as they came.
bool submitPointSetColumnOvStage(::ev4sio_physx::PxVec4* dst,
                                 const float* src,
                                 const PointSetTransform& localToWorld,
                                 ::ev4sio_physx::PxU32 numPoints,
                                 bool reframe);

// A volume-deformable kinematic-target buffer as its setup leaves it (ADR-0012): every node free (w = 1) at its
// position in `positions`, PhysX's sim-mesh positions. On the legacy default stream.
bool initKinematicTargetBufferOvStage(::ev4sio_physx::PxVec4* dst, const ::ev4sio_physx::PxVec4* positions, ::ev4sio_physx::PxU32 numNodes);

// The ovstage volume-deformable KINEMATIC-TARGET commit (ADR-0012, ADR-0052): every body of a group in one launch on
// `stream`, from the caller's column (four floats per node: an xyz target in the sim mesh's frame and `w`, 0 pinning
// the node) into the buffers PhysX adopted. Bodies `mask` leaves out (one byte per body, nonzero selects; null
// selects all) and bodies with no buffer are left as they are. A pinned node the column marks free keeps its target
// and adds one to `keptPinned`, since PhysX never frees a pinned node. Reads no host memory but `keptPinned` when it
// is mapped, so it can be captured into a CUDA graph.
bool submitKinematicTargetsOvStage(const KinematicTargetBodyRecord* bodies,
                                   ::ev4sio_physx::PxU32 numBodies,
                                   ::ev4sio_physx::PxU32 maxNodes,
                                   const float* column,
                                   const uint8_t* mask,
                                   unsigned long long* keptPinned,
                                   cudaStream_t stream);

// The ovstage volume-deformable KINEMATIC-TARGET read: gather one body's runtime-owned PxVec4 buffer
// (the one setKinematicTargetBufferD adopted) into a four-float-per-node column (xyz + w flag). The
// inverse of submitKinematicTargetsOvStage: xyz is carried world->sim-mesh by `worldToLocal`
// exactly as fetchPointSetColumnOvStage reframes points, and the w flag passes through unchanged so a
// reader sees which nodes are driven. Per BODY, not rectangular -- each body a kinematic-target write set up owns
// its own buffer.
bool fetchKinematicTargetColumnOvStage(float* dst,
                                       const ::ev4sio_physx::PxVec4* src,
                                       const PointSetTransform& worldToLocal,
                                       ::ev4sio_physx::PxU32 numNodes,
                                       bool reframe,
                                       cudaStream_t stream);

// Device deg->rad (or any constant) scale of a contiguous float column: dst[i] = k * src[i]. Lets the
// ovstage device drain convert an angular-velocity column device-to-device, with no host round trip.
bool scaleFloatColumnOvStage(float* dst, const float* src, ::ev4sio_physx::PxU32 count, float k, cudaStream_t stream = nullptr);

// Decompose a column-major world transform column (double[16] per prim) into a position (float[3]) and a
// quaternion (float[4], xyzw), device-to-device. Lets the ovstage device pose drain turn authored world
// matrices into the pos/orientation the DirectGPU rigid setters take, with no host round trip. `srcRow`, when
// non-null, is a device gather map (output row i reads storage row srcRow[i]) for an index-mapped column; a
// null `srcRow` reads storage row i (dense). The float[16] overload widens each element to double first, as the
// host decode of a float32 world matrix does, so both overloads decompose in double precision.
bool decomposeWorldMatrixOvStage(const double* mat, const ::ev4sio_physx::PxU32* srcRow, float* pos, float* quat,
                                 ::ev4sio_physx::PxU32 rows);
bool decomposeWorldMatrixOvStage(const float* mat, const ::ev4sio_physx::PxU32* srcRow, float* pos, float* quat,
                                 ::ev4sio_physx::PxU32 rows);

// Top bit of a scatter index: set when that row's value is a wrapped-revolute drive target to fold into
// [-360, 360] on device. The DOF offset lives in the low 31 bits -- the scene's single-DOF count bounds it,
// so it always fits. The scatter takes ONE packed index column: one allocation and one host->device upload
// per drain.
constexpr ::ev4sio_physx::PxU32 kScatterDofWrapBit = 0x80000000u;

// Device scatter of a joint DOF column from change/key order into the session's DOF-order column:
// dst[dstIdx[i] & ~kScatterDofWrapBit] = src[i], with the drive-position [-360, 360] wrap applied on each row
// whose index has kScatterDofWrapBit set (a wrapped-revolute drive target out of range) so the device joint
// drain matches the host path. `dst` must be pre-zeroed by the caller. Lets the ovstage device joint drain
// fill the session column device-to-device, with no host round trip.
bool scatterDofColumnOvStage(float* dst, const float* src, const ::ev4sio_physx::PxU32* dstIdx, ::ev4sio_physx::PxU32 count,
                             cudaStream_t stream = nullptr);

} // namespace tensors
} // namespace ev4sio_physx
} // namespace omni
