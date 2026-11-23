// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-LIFETIME-001
// @covers AC-1 AC-2

// @implements REQ-ATTACH-IO-001
// @covers AC-3

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-4
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-10 AC-11 AC-12
 */

#pragma once

#include <PxPhysicsAPI.h>
#include <map>
#include <mutex>
#include <omni/physx/IPhysxCustomGeometry.h>
#include <common/foundation/Allocator.h>

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/CustomGeometryHash.h>

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <carb/tasking/TaskingUtils.h>

namespace omni
{
namespace ev4sio_physx
{
namespace usdparser { class AttachedStage; }
struct CustomGeometryInfo
{
    // Registered token spelling, kept as a std::string so `computeCustomGeometryHash`
    // (the std::string overload -- the sole overload since ADR-0019 dropped the
    // TfToken one) can be recomputed at unregister time without holding a TfToken.
    std::string customGeomtryAPIToken;
    ICustomGeometryCallback customGeometryCb;
    std::shared_ptr<::ev4sio_physx::PxCustomGeometry::Type> typeId;
};


class PhysXCustomGeometryManager;

enum class GeometryTrackingKind { Unbound, Shape };
struct GeometryTrackingKey
{
    GeometryTrackingKind kind=GeometryTrackingKind::Unbound;
    const void* address=nullptr;
    bool operator<(const GeometryTrackingKey& other) const noexcept
    {
        if (kind!=other.kind) return kind<other.kind;
        return std::less<const void*>()(address,other.address);
    }
};

class CustomPhysXGeometryCallback : public ::ev4sio_physx::PxCustomGeometry::Callbacks, public Allocateable
{
public:
    CustomPhysXGeometryCallback(PhysXCustomGeometryManager& manager, const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key,
                                const usdparser::CustomPhysxShapeDesc& shapeDesc,
                                const CustomGeometryInfo& customGeometryInfo,
                                void* userObject);

    void release();

    // PxCustomGeometry::Callbacks boilerplate
    virtual ::ev4sio_physx::PxCustomGeometry::Type getCustomType() const
    {
        return *mCustomGeometryInfo.typeId;
    }

    ::ev4sio_physx::PxBounds3 getLocalBounds(const ::ev4sio_physx::PxGeometry& geometry) const;

    bool generateContacts(const ::ev4sio_physx::PxGeometry& geom0,
                          const ::ev4sio_physx::PxGeometry& geom1,
                          const ::ev4sio_physx::PxTransform& pose0,
                          const ::ev4sio_physx::PxTransform& pose1,
                          const float contactDistance,
                          const float meshContactMargin,
                          const float toleranceLength,
                          ::ev4sio_physx::PxContactBuffer& contactBuffer) const;

    ::ev4sio_physx::PxU32 raycast(const ::ev4sio_physx::PxVec3& origin,
                           const ::ev4sio_physx::PxVec3& unitDir,
                           const ::ev4sio_physx::PxGeometry& geom,
                           const ::ev4sio_physx::PxTransform& pose,
                           ::ev4sio_physx::PxReal maxDist,
                           ::ev4sio_physx::PxHitFlags hitFlags,
                           ::ev4sio_physx::PxU32 maxHits,
                           ::ev4sio_physx::PxGeomRaycastHit* rayHits,
                           ::ev4sio_physx::PxU32 stride,
                           ::ev4sio_physx::PxRaycastThreadContext* threadContext) const;
    bool overlap(const ::ev4sio_physx::PxGeometry& geom0,
                 const ::ev4sio_physx::PxTransform& pose0,
                 const ::ev4sio_physx::PxGeometry& geom1,
                 const ::ev4sio_physx::PxTransform& pose1,
                 ::ev4sio_physx::PxOverlapThreadContext* threadContext) const;
    bool sweep(const ::ev4sio_physx::PxVec3& unitDir,
               const ::ev4sio_physx::PxReal maxDist,
               const ::ev4sio_physx::PxGeometry& geom0,
               const ::ev4sio_physx::PxTransform& pose0,
               const ::ev4sio_physx::PxGeometry& geom1,
               const ::ev4sio_physx::PxTransform& pose1,
               ::ev4sio_physx::PxGeomSweepHit& sweepHit,
               ::ev4sio_physx::PxHitFlags hitFlags,
               const ::ev4sio_physx::PxReal inflation,
               ::ev4sio_physx::PxSweepThreadContext* threadContext) const;

    void visualize(const ::ev4sio_physx::PxGeometry& geometry,
                   ::ev4sio_physx::PxRenderOutput& out,
                   const ::ev4sio_physx::PxTransform& absPose,
                   const ::ev4sio_physx::PxBounds3& cullbox) const;
    void computeMassProperties(const ::ev4sio_physx::PxGeometry& geometry, ::ev4sio_physx::PxMassProperties& massProperties) const;
    bool usePersistentContactManifold(const ::ev4sio_physx::PxGeometry& geometry, ::ev4sio_physx::PxReal& breakingThreshold) const;

    ~CustomPhysXGeometryCallback();

    void* getUserObject() const
    {
        return mUserObject;
    }

private:
    friend class PhysXCustomGeometryManager;
    PhysXCustomGeometryManager& mManager;
    GeometryTrackingKey mTrackingKey;
    const usdparser::AttachedStage& mAttachedStage;
    omni::physics::parse::ObjectKey mCustomGeometryKey;
    CustomGeometryInfo mCustomGeometryInfo;
    void* mUserObject;
};

using CustomGeometryRegistryMap = std::unordered_map<size_t, CustomGeometryInfo>;
// Keyed by computeCustomGeometryHash(the registered token string) so the descriptor
// side can produce the same key from either the registered token (legacy) or a parse-
// lib TokenId resolved through `tokenToString` (USD-free callers).
using CustomGeometryTypeMap = std::unordered_map<size_t, CustomGeometryInfo>;
using CustomGeometryMap =
    std::unordered_multimap<omni::physics::parse::ObjectKey, CustomPhysXGeometryCallback*, omni::physics::parse::ObjectKey::Hash>;

class PhysXCustomGeometryManager : public ::ev4sio_physx::PxDeletionListener
{
public:
    PhysXCustomGeometryManager();
    ~PhysXCustomGeometryManager();

    size_t registerCustomGeometry(const char* customGeometryAPIToken, ICustomGeometryCallback& geometryCallback);
    void unregisterCustomGeometry(size_t id);
    bool tryUnregisterCustomGeometry(size_t id);

    // SDK setup registers this one listener before publishing any shapes. Each
    // provisional geometry preallocates a tracking node; binding is allocation-free.
    void bindShape(CustomPhysXGeometryCallback&, const ::ev4sio_physx::PxShape&);
    void forgetGeometry(CustomPhysXGeometryCallback&) noexcept;
    void releaseUnboundGeometry(CustomPhysXGeometryCallback*);
    void onRelease(const ::ev4sio_physx::PxBase*, void*, ::ev4sio_physx::PxDeletionEventFlag::Enum) override;

    void clear();
    void clear(const usdparser::AttachedStage& owner);

    CustomPhysXGeometryCallback* createCustomGeometry(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey primKey,
                                                      const usdparser::CustomPhysxShapeDesc& customShapeDesc);

    void removeCustomGeometry(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey primKey);

    CustomGeometryTypeMap getCustomGeometryTypeMap() const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        return mCustomGeometryTypeMap;
    }

    bool isKnownType(const ::ev4sio_physx::PxCustomGeometry::Type& customType) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
        for (const std::weak_ptr<::ev4sio_physx::PxCustomGeometry::Type>& entry : mKnownTypes)
        {
            const std::shared_ptr<::ev4sio_physx::PxCustomGeometry::Type> live = entry.lock();
            if (live && *live == customType)
                return true;
        }
        return false;
    }

private:
    std::mutex mShapesMutex;
    std::map<GeometryTrackingKey, CustomPhysXGeometryCallback*> mShapes;
    std::vector<std::weak_ptr<::ev4sio_physx::PxCustomGeometry::Type>> mKnownTypes;
    size_t mGeometryRegistryCounter;
    CustomGeometryRegistryMap mCustomGeometryRegistryMap;
    CustomGeometryTypeMap mCustomGeometryTypeMap;
    mutable carb::tasking::MutexWrapper mRegistryMutex;
};
} // namespace ev4sio_physx
} // namespace omni
