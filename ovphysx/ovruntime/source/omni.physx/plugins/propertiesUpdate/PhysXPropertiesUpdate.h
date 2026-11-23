// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PROPS-MAT-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 */

/**
 * @implements REQ-SIM-BODY-INPUT-001
 * @covers AC-2 AC-5 AC-6
 */

#pragma once

#include <omni/physics/parse/IPhysicsSource.h>

#include <internal/Internal.h>

namespace omni
{
namespace ev4sio_physx
{
namespace usdparser
{
class AttachedStage;
}

// Did the user actually author one of the PhysxCollisionAPI offsets
// (physxCollision:contactOffset / :restOffset), as opposed to inheriting the
// schema fallback? The schema default for both is NEGATIVE inf, an "unset"
// sentinel, so the resolved value answers this without a separate authored bit.
//
// Use this rather than hasAuthoredAttribute in the Newton fallback handlers: a
// resolved-value backend answers hasAuthoredAttribute `true` for everything it
// publishes (ADR-0020), which made every Newton contactMargin/contactGap
// fallback silently inert on ovstage. Defined in PhysXCollisionPropertiesUpdate.cpp.
bool isPhysxOffsetAuthored(const omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           const omni::physics::parse::ObjectKey& key,
                           omni::physics::parse::TokenId attr);

// scene
bool updateGravityMagnitude(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateGravityDirection(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateTimeStepsPerSecond(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateSceneUpdateType(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           omni::ev4sio_physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateQuasistaticEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateQuasistaticCollection(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);

// body
bool updateBodyEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updateBodyDensity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updateBodyLinearVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyAngularVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
// Apply an already captured input without reading the source again.
bool updateBodyLinearVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              const carb::Float3& value);
bool updateBodyAngularVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               const carb::Float3& value);
bool updateBodyLinearDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodyAngularDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyMaxLinearVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateBodyMaxAngularVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId objectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateBodySleepThreshold(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyStabilizationThreshold(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateBodyMaxDepenetrationVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateBodyContactSlopCoefficient(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId property,
                                      omni::physics::parse::ReadTime);
bool updateBodyMaxContactImpulse(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId objectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateBodySolverPositionIterationCount(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateBodySolverVelocityIterationCount(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateBodyEnableKinematics(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateBodyEnableCCD(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateBodyEnableSpeculativeCCD(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId objectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateBodyRetainAccelerations(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateBodyGyroscopicForces(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateBodyDisableGravity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyLockedPosAxis(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodyLockedRotAxis(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodyTransformStack(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateBodyCfmScale(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateBodySolveContacts(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateBodySimulationOwner(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);

bool updatePhysxContactReportThreshold(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);

bool updateBodyInstancedPositions(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId objectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateBodyInstancedOrientations(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateBodyInstancedVelocities(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateBodyInstancedAngularVelocities(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId objectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);

bool updateBodySurfaceVelocityEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateBodySurfaceLinearVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateBodySurfaceAngularVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateBodySurfaceVelocityLocalSpace(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateBodySplineSurfaceVelocityMagnitude(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateBodySplineSurfaceVelocityEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId,
                                              omni::physics::parse::ReadTime);

    // force
bool updatePhysxForceEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updatePhysxForceWorldFrameEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updatePhysxForce(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updatePhysxTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updatePhysxForceMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                          omni::ev4sio_physx::usdparser::ObjectId objectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);

// shape
bool updateShapeEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateShapeDensity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateShapeContactOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateShapeRestOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           omni::ev4sio_physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateShapeTorsionalPatchRadius(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateShapeMinTorsionalPatchRadius(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);

// material
bool updateShapeMaterialBinding(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateMaterialDynamicFriction(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateMaterialStaticFriction(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId objectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateMaterialRestitution(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateMaterialFrictionCombineMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateMaterialRestitutionCombineMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId objectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateMaterialDampingCombineMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateCompliantMaterial(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId property,
                             omni::physics::parse::ReadTime);

// joint
bool updateDriveTargetPosition(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateDriveTargetVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateDriveMaxForce(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateDriveMaxActuatorVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveVelocityDependentResistance(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveSpeedEffortGradient(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateDriveStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                          omni::ev4sio_physx::usdparser::ObjectId objectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateDriveType(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLimitHigh(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLimitLow(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                    omni::ev4sio_physx::usdparser::ObjectId objectId,
                    omni::physics::parse::TokenId,
                    omni::physics::parse::ReadTime);
bool updateJointStatePosition(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateJointStateVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateEnableCollision(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           omni::ev4sio_physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateBreakForce(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updateBreakTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                       omni::physics::parse::TokenId,
                       omni::physics::parse::ReadTime);
bool updateLocalPos0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLocalPos1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLocalRot0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateLocalRot1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateGearRatio(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateGearHinge0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updateGearHinge1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                      omni::physics::parse::TokenId,
                      omni::physics::parse::ReadTime);
bool updateRackPinionRatio(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           omni::ev4sio_physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);
bool updateRackHinge(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateRackPrismatic(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateArticulationArmature(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateLimitBounceThreshold(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateLimitDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                        omni::physics::parse::TokenId,
                        omni::physics::parse::ReadTime);
bool updateLimitRestitution(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateLimitStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                          omni::ev4sio_physx::usdparser::ObjectId objectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateDistanceJointSpringDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateDistanceJointSpringStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateDistanceJointSpringEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);

// articulation
bool updateArticulationFixBase(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId objectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateArticulationSolverPositionIterationCount(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                    omni::ev4sio_physx::usdparser::ObjectId objectId,
                                                    omni::physics::parse::TokenId,
                                                    omni::physics::parse::ReadTime);
bool updateArticulationSolverVelocityIterationCount(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                    omni::ev4sio_physx::usdparser::ObjectId objectId,
                                                    omni::physics::parse::TokenId,
                                                    omni::physics::parse::ReadTime);
bool updateArticulationSleepThreshold(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId objectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateArticulationStabilizationThreshold(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId,
                                              omni::physics::parse::ReadTime);

// articulation link
bool updateArticulationMaxJointVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId objectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateArticulationStaticFrictionEffort(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateArticulationDynamicFrictionEffort(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);
bool updateArticulationViscousFrictionCoefficient(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

// collision group
inline bool updateCollisionGroup(omni::ev4sio_physx::usdparser::AttachedStage& /*attachedStage*/,
                                 omni::ev4sio_physx::usdparser::ObjectId /*objectId*/,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime)
{
    return true;
}

// filtered pairs
bool updateFilteredPairs(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);

// cct
bool updateCctSlopeLimit(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateCctHeight(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateCctRadius(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateCctContactOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateCctStepOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                         omni::ev4sio_physx::usdparser::ObjectId objectId,
                         omni::physics::parse::TokenId,
                         omni::physics::parse::ReadTime);
bool updateCctUpAxis(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                     omni::physics::parse::TokenId,
                     omni::physics::parse::ReadTime);
bool updateCctNonWalkableMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateCctClimbingMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           omni::ev4sio_physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);

// particle system
bool updateParticleSystemAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updatePBDMaterialAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateParticleDensity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                           omni::ev4sio_physx::usdparser::ObjectId objectId,
                           omni::physics::parse::TokenId,
                           omni::physics::parse::ReadTime);

bool updateParticleSmoothingEnabledAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                                             omni::physics::parse::TokenId property,
                                             omni::physics::parse::ReadTime);
bool updateParticleAnisotropyEnabledAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId property,
                                              omni::physics::parse::ReadTime);
bool updateParticleIsosurfaceEnabledAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                                              omni::physics::parse::TokenId property,
                                              omni::physics::parse::ReadTime);
bool updateParticleIsosurfaceAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId objectId,
                                       omni::physics::parse::TokenId property,
                                       omni::physics::parse::ReadTime);

// partice set
bool updateParticleSetEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateParticleSetSelfCollision(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId objectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateParticleSetFluid(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateParticleSetParticleGroup(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId objectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);

bool updateParticlePositions(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId objectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateParticleSimPositions(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateParticleVelocities(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);

bool updateDiffuseParticlesEnabledAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId objectId,
                                            omni::physics::parse::TokenId property,
                                            omni::physics::parse::ReadTime);
bool updateDiffuseParticlesAttribute(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId objectId,
                                     omni::physics::parse::TokenId property,
                                     omni::physics::parse::ReadTime);

// vehicle
bool updateVehicleContextUpdateMode(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateVehicleContextVerticalAxis(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleContextLongitudinalAxis(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);

bool updateVehicleEngineMomentOfInertia(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleEnginePeakTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleEngineMaxRotationSpeed(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleEngineIdleRotationSpeed(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleEngineTorqueCurve(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateVehicleEngineDampingRateFullThrottle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                omni::ev4sio_physx::usdparser::ObjectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);
bool updateVehicleEngineDampingRateZeroThrottleClutchEngaged(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                             omni::ev4sio_physx::usdparser::ObjectId,
                                                             omni::physics::parse::TokenId,
                                                             omni::physics::parse::ReadTime);
bool updateVehicleEngineDampingRateZeroThrottleClutchDisengaged(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                                omni::ev4sio_physx::usdparser::ObjectId,
                                                                omni::physics::parse::TokenId,
                                                                omni::physics::parse::ReadTime);

bool updateVehicleTireFrictionTableFrictionValues(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                  omni::ev4sio_physx::usdparser::ObjectId,
                                                  omni::physics::parse::TokenId,
                                                  omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionTableGroundMaterials(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                   omni::ev4sio_physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionTableDefaultFrictionValue(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                        omni::ev4sio_physx::usdparser::ObjectId,
                                                        omni::physics::parse::TokenId,
                                                        omni::physics::parse::ReadTime);

bool updateVehicleSuspensionSpringStrength(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleSuspensionSpringDamperRate(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                             omni::ev4sio_physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleSuspensionMaxCompression(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleSuspensionMaxDroop(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateVehicleSuspensionTravelDistance(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleSuspensionSprungMass(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleSuspensionCamberAtRest(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleSuspensionCamberAtMaxCompression(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                   omni::ev4sio_physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleSuspensionCamberAtMaxDroop(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                             omni::ev4sio_physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);

bool updateVehicleTireLatStiffX(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleTireLatStiffY(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleTireLateralStiffnessGraph(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateVehicleTireLongStiffPerGrav(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleTireLongitudinalStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateVehicleTireCamberStiffPerGrav(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleTireCamberStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionVsSlip(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateVehicleTireFrictionTableRel(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleTireRestLoad(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);

bool updateVehicleWheelRadius(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateVehicleWheelWidth(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateVehicleWheelMass(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateVehicleWheelMomentOfInertia(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelDampingRate(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleWheelMaxBrakeTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleWheelMaxHandBrakeTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleWheelMaxSteerAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateVehicleWheelToeAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);

bool updateVehicleWheelAttachmentIndex(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheel(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentTire(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspension(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionTravelDirection(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                           omni::ev4sio_physx::usdparser::ObjectId,
                                                           omni::physics::parse::TokenId,
                                                           omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionForceAppPointOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                               omni::ev4sio_physx::usdparser::ObjectId,
                                                               omni::physics::parse::TokenId,
                                                               omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheelCenterOfMassOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                         omni::ev4sio_physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentTireForceAppPointOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                         omni::ev4sio_physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionFramePosition(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                         omni::ev4sio_physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentSuspensionFrameOrientation(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                            omni::ev4sio_physx::usdparser::ObjectId,
                                                            omni::physics::parse::TokenId,
                                                            omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheelFramePosition(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                    omni::ev4sio_physx::usdparser::ObjectId,
                                                    omni::physics::parse::TokenId,
                                                    omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentWheelFrameOrientation(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                       omni::ev4sio_physx::usdparser::ObjectId,
                                                       omni::physics::parse::TokenId,
                                                       omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentDriven(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleWheelAttachmentCollisionGroup(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                omni::ev4sio_physx::usdparser::ObjectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);

bool updateVehicleSuspensionComplWheelToeAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                               omni::ev4sio_physx::usdparser::ObjectId,
                                               omni::physics::parse::TokenId,
                                               omni::physics::parse::ReadTime);
bool updateVehicleSuspensionComplWheelCamberAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                  omni::ev4sio_physx::usdparser::ObjectId,
                                                  omni::physics::parse::TokenId,
                                                  omni::physics::parse::ReadTime);
bool updateVehicleSuspensionComplSuspForceAppPoint(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                   omni::ev4sio_physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleSuspensionComplTireForceAppPoint(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                   omni::ev4sio_physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);

bool updateVehicleEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                          omni::ev4sio_physx::usdparser::ObjectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateVehicleLimitSuspensionExpansionVelocity(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                   omni::ev4sio_physx::usdparser::ObjectId,
                                                   omni::physics::parse::TokenId,
                                                   omni::physics::parse::ReadTime);
bool updateVehicleMinPassiveLongslipDenom(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleMinActiveLongslipDenom(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleMinLateralSlipDenom(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleLongitudinalStickyTireThresholdSpeed(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                       omni::ev4sio_physx::usdparser::ObjectId,
                                                       omni::physics::parse::TokenId,
                                                       omni::physics::parse::ReadTime);
bool updateVehicleLongitudinalStickyTireThresholdTime(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                      omni::ev4sio_physx::usdparser::ObjectId,
                                                      omni::physics::parse::TokenId,
                                                      omni::physics::parse::ReadTime);
bool updateVehicleLongitudinalStickyTireDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                omni::ev4sio_physx::usdparser::ObjectId,
                                                omni::physics::parse::TokenId,
                                                omni::physics::parse::ReadTime);
bool updateVehicleLateralStickyTireThresholdSpeed(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                  omni::ev4sio_physx::usdparser::ObjectId,
                                                  omni::physics::parse::TokenId,
                                                  omni::physics::parse::ReadTime);
bool updateVehicleLateralStickyTireThresholdTime(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                 omni::ev4sio_physx::usdparser::ObjectId,
                                                 omni::physics::parse::TokenId,
                                                 omni::physics::parse::ReadTime);
bool updateVehicleLateralStickyTireDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

bool updateVehicleControllerAccelerator(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleControllerBrake0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleControllerBrake1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateVehicleControllerBrake(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateVehicleControllerHandbrake(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleControllerSteer(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateVehicleControllerSteerLeft(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateVehicleControllerSteerRight(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateVehicleControllerTargetGear(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);

bool updateVehicleTankControllerThrust0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleTankControllerThrust1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);

bool updateVehicleDriveBasicPeakTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);

bool updateVehicleWheelControllerDriveTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                             omni::ev4sio_physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleWheelControllerBrakeTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                             omni::ev4sio_physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleWheelControllerSteerAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

bool updateVehicleMultiWheelDifferentialWheels(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                               omni::ev4sio_physx::usdparser::ObjectId,
                                               omni::physics::parse::TokenId,
                                               omni::physics::parse::ReadTime);
bool updateVehicleMultiWheelDifferentialTorqueRatios(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                     omni::ev4sio_physx::usdparser::ObjectId,
                                                     omni::physics::parse::TokenId,
                                                     omni::physics::parse::ReadTime);
bool updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                                omni::ev4sio_physx::usdparser::ObjectId,
                                                                omni::physics::parse::TokenId,
                                                                omni::physics::parse::ReadTime);

bool updateVehicleTankDifferentialNumberOfWheelsPerTrack(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                         omni::ev4sio_physx::usdparser::ObjectId,
                                                         omni::physics::parse::TokenId,
                                                         omni::physics::parse::ReadTime);
bool updateVehicleTankDifferentialThrustIndexPerTrack(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                      omni::ev4sio_physx::usdparser::ObjectId,
                                                      omni::physics::parse::TokenId,
                                                      omni::physics::parse::ReadTime);
bool updateVehicleTankDifferentialTrackToWheelIndices(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                      omni::ev4sio_physx::usdparser::ObjectId,
                                                      omni::physics::parse::TokenId,
                                                      omni::physics::parse::ReadTime);
bool updateVehicleTankDifferentialWheelIndicesInTrackOrder(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                           omni::ev4sio_physx::usdparser::ObjectId,
                                                           omni::physics::parse::TokenId,
                                                           omni::physics::parse::ReadTime);

bool updateVehicleBrakes0Wheels(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleBrakes1Wheels(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateVehicleBrakes0MaxBrakeTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleBrakes1MaxBrakeTorque(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleBrakes0TorqueMultipliers(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleBrakes1TorqueMultipliers(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

bool updateVehicleSteeringWheels(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateVehicleSteeringMaxSteerAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleSteeringAngleMultipliers(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

bool updateVehicleAckermannSteeringWheel0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringWheel1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringMaxSteerAngle(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                 omni::ev4sio_physx::usdparser::ObjectId,
                                                 omni::physics::parse::TokenId,
                                                 omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringWheelBase(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                             omni::ev4sio_physx::usdparser::ObjectId,
                                             omni::physics::parse::TokenId,
                                             omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringTrackWidth(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                              omni::ev4sio_physx::usdparser::ObjectId,
                                              omni::physics::parse::TokenId,
                                              omni::physics::parse::ReadTime);
bool updateVehicleAckermannSteeringStrength(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

bool updateVehicleNCRDriveCommandValues(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRSteerCommandValues(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes0CommandValues(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes1CommandValues(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);

bool updateVehicleNCRDriveSpeedResponsesPerCommandValue(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                        omni::ev4sio_physx::usdparser::ObjectId,
                                                        omni::physics::parse::TokenId,
                                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRSteerSpeedResponsesPerCommandValue(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                        omni::ev4sio_physx::usdparser::ObjectId,
                                                        omni::physics::parse::TokenId,
                                                        omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes0SpeedResponsesPerCommandValue(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                          omni::ev4sio_physx::usdparser::ObjectId,
                                                          omni::physics::parse::TokenId,
                                                          omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes1SpeedResponsesPerCommandValue(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                                          omni::ev4sio_physx::usdparser::ObjectId,
                                                          omni::physics::parse::TokenId,
                                                          omni::physics::parse::ReadTime);

bool updateVehicleNCRDriveSpeedResponses(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleNCRSteerSpeedResponses(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes0SpeedResponses(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);
bool updateVehicleNCRBrakes1SpeedResponses(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                           omni::ev4sio_physx::usdparser::ObjectId,
                                           omni::physics::parse::TokenId,
                                           omni::physics::parse::ReadTime);

// spatial tendons
bool updateSpatialTendonStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);
bool updateSpatialTendonDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateSpatialTendonLimitStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                       omni::ev4sio_physx::usdparser::ObjectId,
                                       omni::physics::parse::TokenId,
                                       omni::physics::parse::ReadTime);
bool updateSpatialTendonOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateSpatialTendonEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateTendonAttachmentGearing(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateTendonAttachmentLocalPos(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateTendonAttachmentLeafRestLength(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                          omni::ev4sio_physx::usdparser::ObjectId,
                                          omni::physics::parse::TokenId,
                                          omni::physics::parse::ReadTime);
bool updateTendonAttachmentLeafLowLimit(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                        omni::ev4sio_physx::usdparser::ObjectId,
                                        omni::physics::parse::TokenId,
                                        omni::physics::parse::ReadTime);
bool updateTendonAttachmentLeafHighLimit(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);

// fixed tendons
bool updateFixedTendonStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateFixedTendonLimitStiffness(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                     omni::ev4sio_physx::usdparser::ObjectId,
                                     omni::physics::parse::TokenId,
                                     omni::physics::parse::ReadTime);
bool updateFixedTendonDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateFixedTendonOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateFixedTendonRestLength(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateFixedTendonLowLimit(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateFixedTendonHighLimit(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateFixedTendonEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);
bool updateTendonAxisSingleGearing(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateTendonAxisSingleForceCoefficient(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                            omni::ev4sio_physx::usdparser::ObjectId,
                                            omni::physics::parse::TokenId,
                                            omni::physics::parse::ReadTime);

// deformables
bool updateDeformableBody(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                          omni::ev4sio_physx::usdparser::ObjectId,
                          omni::physics::parse::TokenId,
                          omni::physics::parse::ReadTime);
bool updateDeformableContactOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                   omni::ev4sio_physx::usdparser::ObjectId objectId,
                                   omni::physics::parse::TokenId,
                                   omni::physics::parse::ReadTime);
bool updateDeformableRestOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId objectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateDeformableMaterial(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId objectId,
                              omni::physics::parse::TokenId property,
                              omni::physics::parse::ReadTime);

// mimic joints
bool updateMimicJointGearing(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                             omni::ev4sio_physx::usdparser::ObjectId,
                             omni::physics::parse::TokenId,
                             omni::physics::parse::ReadTime);
bool updateMimicJointOffset(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                            omni::ev4sio_physx::usdparser::ObjectId,
                            omni::physics::parse::TokenId,
                            omni::physics::parse::ReadTime);
bool updateMimicJointNaturalFrequency(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateMimicJointDampingRatio(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                  omni::ev4sio_physx::usdparser::ObjectId,
                                  omni::physics::parse::TokenId,
                                  omni::physics::parse::ReadTime);

bool updateNewtonMimicJointCoef1(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateNewtonMimicJointCoef0(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);

// note: the other mimic joint properties trigger a structural change, so a reparse, thus no change methods for those

// Newton schema runtime change tracking. Each wrapper applies the Newton value only when the
// corresponding PhysX attribute is not authored on the prim (PhysX > Newton priority).
bool updateNewtonTimeStepsPerSecond(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateNewtonGravityEnabled(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                omni::ev4sio_physx::usdparser::ObjectId,
                                omni::physics::parse::TokenId,
                                omni::physics::parse::ReadTime);
bool updateNewtonShapeContactMargin(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateNewtonShapeContactGap(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                 omni::ev4sio_physx::usdparser::ObjectId,
                                 omni::physics::parse::TokenId,
                                 omni::physics::parse::ReadTime);
bool updateNewtonDeformableContactMargin(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                         omni::ev4sio_physx::usdparser::ObjectId,
                                         omni::physics::parse::TokenId,
                                         omni::physics::parse::ReadTime);
bool updateNewtonDeformableContactGap(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                      omni::ev4sio_physx::usdparser::ObjectId,
                                      omni::physics::parse::TokenId,
                                      omni::physics::parse::ReadTime);
bool updateNewtonJointVelocityLimit(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                                    omni::ev4sio_physx::usdparser::ObjectId,
                                    omni::physics::parse::TokenId,
                                    omni::physics::parse::ReadTime);
bool updateNewtonJointArmature(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateNewtonJointFriction(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                               omni::ev4sio_physx::usdparser::ObjectId,
                               omni::physics::parse::TokenId,
                               omni::physics::parse::ReadTime);
bool updateNewtonJointDamping(omni::ev4sio_physx::usdparser::AttachedStage& attachedStage,
                              omni::ev4sio_physx::usdparser::ObjectId,
                              omni::physics::parse::TokenId,
                              omni::physics::parse::ReadTime);

} // namespace ev4sio_physx
} // namespace omni
