#include "PxPhysicsAPI.h"
#include "PxImmediateMode.h"
#include "vehicle/PxVehicleAPI.h"
#include "vehicle/PxVehicleEv4sioCallbacks.h"
#include "ExtJointData.h"
#include "NvBlastExtAuthoringTypes.h"
#include <cstdio>
#include <type_traits>
#ifdef _WIN32
#include <windows.h>
#define EV4SIO_TEST_EXPORT __declspec(dllexport)
#else
#define EV4SIO_TEST_EXPORT __attribute__((visibility("default")))
#endif

using namespace ev4sio_physx;

static_assert(std::is_destructible<ev4sio_Ext::JointData>::value, "Joint data must be accessible to the plugin");

struct TestErrors : PxErrorCallback
{
    unsigned errors = 0;
    void reportError(PxErrorCode::Enum code, const char* message, const char*, int) override
    {
        std::fprintf(stderr, "PhysX [%d]: %s\n", int(code), message);
        if (code != PxErrorCode::eDEBUG_INFO && code != PxErrorCode::eDEBUG_WARNING && code != PxErrorCode::ePERF_WARNING)
            ++errors;
    }
};

static unsigned tireCalls = 0;
static bool tireInputsValid = false;
static void tireShader(const void* data, PxF32 friction, PxF32, PxF32, PxF32,
    PxF32 longSpeed, PxF32 latSpeed, PxF32 omega, PxF32 radius, PxF32,
    PxF32 restLoad, PxF32, PxF32 load, PxF32 gravity, PxF32,
    PxF32& torque, PxF32& longForce, PxF32& latForce, PxF32& align)
{
    ++tireCalls;
    tireInputsValid = data && friction == 1.0f && longSpeed == 13.25f && latSpeed == -2.5f &&
        omega == 32.0f && radius == 0.5f && restLoad == 4000.0f && load == 4000.0f && gravity == 9.81f;
    torque = -20.0f; longForce = 100.0f; latForce = -50.0f; align = 7.0f;
}

static bool testTires()
{
    PxVehicleWheelParams wheel = {};
    wheel.radius = 0.5f; wheel.halfWidth = 0.2f;
    PxVehicleSuspensionParams suspension = {};
    suspension.suspensionAttachment = PxTransform(PxIdentity);
    PxVehicleSuspensionComplianceState compliance = {};
    PxVehicleTireForceParams params = {};
    params.latStiffX = 2.0f; params.latStiffY = 80000.0f; params.longStiff = 100000.0f;
    params.restLoad = 4000.0f;
    PxVehicleTireGripState grip = {};
    grip.friction = 1.0f; grip.load = 4000.0f;
    PxVehicleTireDirectionState directions = {};
    directions.directions[0] = PxVec3(1, 0, 0);
    directions.directions[1] = PxVec3(0, 0, 1);
    PxVehicleTireSlipState slips = {};
    slips.slips[0] = 0.1f; slips.slips[1] = 0.02f;
    PxVehicleTireCamberAngleState camber = {};
    PxVehicleRigidBodyState rigid = {};
    rigid.pose = PxTransform(PxIdentity);
    PxVehicleTireSpeedState speeds = {};
    speeds.speedStates[0] = 13.25f; speeds.speedStates[1] = -2.5f;
    PxVehicleTireForce baseline, actual;
    PxVehicleTireForcesUpdate(wheel, suspension, params, compliance, grip, directions, slips, camber, rigid, baseline);
    params.tireForceShader = ev4sio_PxVehicleComputeTireForceDefault;
    PxVehicleTireForcesUpdate(wheel, suspension, params, compliance, grip, directions, slips, camber, rigid,
        actual, &speeds, 32.0f, 9.81f);
    if (PxAbs(baseline.wheelTorque - actual.wheelTorque) > 1e-4f ||
        (baseline.forces[0] - actual.forces[0]).magnitude() > 1e-4f ||
        (baseline.forces[1] - actual.forces[1]).magnitude() > 1e-4f)
        return false;
    params.tireForceShader = tireShader;
    PxVehicleTireForcesUpdate(wheel, suspension, params, compliance, grip, directions, slips, camber, rigid,
        actual, &speeds, 32.0f, 9.81f);
    if (!tireInputsValid || tireCalls != 1 || actual.wheelTorque != -20.0f || actual.aligningMoment != 7.0f ||
        actual.forces[0].x != 100.0f || actual.forces[1].z != -50.0f)
        return false;
    grip.load = 0.0f;
    PxVehicleTireForcesUpdate(wheel, suspension, params, compliance, grip, directions, slips, camber, rigid,
        actual, &speeds, 32.0f, 9.81f);
    return tireCalls == 1 && actual.wheelTorque == 0.0f;
}

static unsigned sweepCalls = 0;
static bool sweepInputsValid = false;
static void sweepShader(const void* data, const PxVec3&, const PxVec3& direction,
    PxF32 width, PxF32 radius, PxSweepHit& hit, PxRigidActor* actor)
{
    ++sweepCalls;
    sweepInputsValid = data && actor && direction.isNormalized() && width == 0.4f && radius == 0.5f;
    hit.position.y += 0.125f;
}

extern "C" EV4SIO_TEST_EXPORT int ev4sio_run_smoke_tests()
{
    PxDefaultAllocator allocator;
    TestErrors errors;
    PxFoundation* foundation = ev4sio_PxCreateFoundation(ev4sio_PX_PHYSICS_VERSION, allocator, errors);
    if (!foundation) return 1;
    PxPhysics* physics = ev4sio_PxCreatePhysics(ev4sio_PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
    if (!physics || !ev4sio_PxInitExtensions(*physics, NULL)) return 2;
    bool ok = testTires();
    std::printf("Tire callbacks: %s\n", ok ? "PASS" : "FAIL");
    Nv::Blast::ConvexDecompositionParams decomposition;
    ok = ok && !decomposition.convexHullApproximation && decomposition.planeDownsampling == 4 &&
        decomposition.convexHullDownsampling == 4 && !decomposition.principalComponentAnalysis;
    PxGpuDynamicsMemoryConfig gpuMemory;
    ok = ok && gpuMemory.isValid();
    PxSolverBodyData solverBody;
    immediate::ev4sio_PxConstructStaticSolverBody(PxTransform(PxIdentity), solverBody);

    PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(2);
    PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0, -9.81f, 0);
    sceneDesc.cpuDispatcher = dispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    PxScene* scene = physics->createScene(sceneDesc);
    PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.1f);
    PxRigidStatic* plane = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
    PxRigidDynamic* sphere = PxCreateDynamic(*physics, PxTransform(PxVec3(0, 5, 0)), PxSphereGeometry(0.5f), *material, 1.0f);
    scene->addActor(*plane); scene->addActor(*sphere);
    for (unsigned i = 0; i < 240; ++i) { scene->simulate(1.0f / 60.0f); scene->fetchResults(true); }
    ok = ok && PxAbs(sphere->getGlobalPose().p.y - 0.5f) < 0.03f;

    const PxVec3 vertices[] = {PxVec3(-1,-1,-1), PxVec3(1,-1,-1), PxVec3(-1,1,-1), PxVec3(1,1,-1),
        PxVec3(-1,-1,1), PxVec3(1,-1,1), PxVec3(-1,1,1), PxVec3(1,1,1)};
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = 8; convexDesc.points.stride = sizeof(PxVec3); convexDesc.points.data = vertices;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    PxCookingParams cooking(physics->getTolerancesScale());
    PxConvexMesh* mesh = ev4sio_PxCreateConvexMesh(cooking, convexDesc, physics->getPhysicsInsertionCallback());
    ok = ok && mesh;
    if (mesh)
    {
        PxVehicleWheelParams wheel = {}; wheel.radius = 0.5f; wheel.halfWidth = 0.2f;
        PxVehicleSuspensionParams suspension = {};
        suspension.suspensionAttachment = PxTransform(PxIdentity);
        suspension.wheelAttachment = PxTransform(PxIdentity);
        suspension.suspensionTravelDir = PxVec3(0,-1,0); suspension.suspensionTravelDist = 3.0f;
        PxVehicleRigidBodyState rigid = {}; rigid.pose = PxTransform(PxVec3(3,2,0));
        PxVehicleFrame frame; frame.setToDefault();
        PxVehiclePhysXMaterialFrictionParams friction = {}; friction.defaultFriction = 1.0f;
        PxVehicleRoadGeometryState road;
        PxVehiclePhysXRoadGeometryQueryState hit;
        PxQueryFilterData filter(PxQueryFlag::eSTATIC);
        PxVehiclePhysXRoadGeometryQueryUpdate(wheel, suspension, PxVehiclePhysXRoadGeometryQueryType::eSWEEP,
            NULL, filter, friction, 0.0f, rigid, *scene, mesh, frame, road, &hit, sweepShader, &wheel);
        ok = ok && road.hitState && sweepCalls == 1 && sweepInputsValid && PxAbs(road.plane.d + 0.125f) < 1e-4f;
        // Exercise overlap recovery, then verify raycasts do not call the sweep shader.
        rigid.pose.p.y = 0.1f;
        PxVehiclePhysXRoadGeometryQueryUpdate(wheel, suspension, PxVehiclePhysXRoadGeometryQueryType::eSWEEP,
            NULL, filter, friction, 0.0f, rigid, *scene, mesh, frame, road, &hit, sweepShader, &wheel);
        ok = ok && road.hitState && sweepCalls == 2 && PxAbs(road.plane.d + 0.125f) < 1e-4f;
        rigid.pose.p.y = 2.0f;
        PxVehiclePhysXRoadGeometryQueryUpdate(wheel, suspension, PxVehiclePhysXRoadGeometryQueryType::eRAYCAST,
            NULL, filter, friction, 0.0f, rigid, *scene, NULL, frame, road, &hit, sweepShader, &wheel);
        ok = ok && road.hitState && sweepCalls == 2 && PxAbs(road.plane.d) < 1e-4f;
        mesh->release();
    }

#if defined(EV4SIO_TEST_GPU) && defined(_WIN32)
    HMODULE driver = LoadLibraryW(L"nvcuda.dll");
    if (driver)
    {
        typedef int (__stdcall *CuInit)(unsigned);
        CuInit init = reinterpret_cast<CuInit>(GetProcAddress(driver, "cuInit"));
        if (init && init(0) == 0)
        {
            PxCudaContextManagerDesc desc;
            PxCudaContextManager* context = ev4sio_PxCreateCudaContextManager(*foundation, desc);
            ok = ok && context && context->contextIsValid();
            if (context && context->contextIsValid())
            {
                PxSceneDesc gpuDesc(physics->getTolerancesScale());
                gpuDesc.gravity = sceneDesc.gravity;
                gpuDesc.cpuDispatcher = dispatcher;
                gpuDesc.filterShader = PxDefaultSimulationFilterShader;
                gpuDesc.cudaContextManager = context;
                gpuDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS | PxSceneFlag::eENABLE_PCM;
                gpuDesc.broadPhaseType = PxBroadPhaseType::eGPU;
                PxScene* gpuScene = physics->createScene(gpuDesc);
                ok = ok && gpuScene;
                if (gpuScene)
                {
                    PxRigidStatic* gpuPlane = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
                    PxRigidDynamic* box = PxCreateDynamic(*physics, PxTransform(PxVec3(0, 3, 0)),
                        PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, 1.0f);
                    gpuScene->addActor(*gpuPlane); gpuScene->addActor(*box);
                    for (unsigned i = 0; i < 240; ++i)
                    {
                        gpuScene->simulate(1.0f / 60.0f);
                        gpuScene->fetchResults(true);
                    }
                    const bool landed = PxAbs(box->getGlobalPose().p.y - 0.5f) < 0.03f;
                    ok = ok && landed;
                    std::printf("GPU simulation: %s (y=%.4f)\n", landed ? "PASS" : "FAIL", box->getGlobalPose().p.y);
                    box->release(); gpuPlane->release(); gpuScene->release();
                }
            }
            if (context) context->release();
            std::puts("GPU context checked.");
        }
        else std::puts("GPU runtime skipped: no available CUDA device.");
        FreeLibrary(driver);
    }
    else std::puts("GPU runtime skipped: NVIDIA driver not installed.");
#endif
    sphere->release(); plane->release(); scene->release(); dispatcher->release(); material->release();
    ev4sio_PxCloseExtensions(); physics->release(); foundation->release();
    ok = ok && errors.errors == 0;
    std::printf("Ev4sio SDK integration: %s (%u errors, %u tire calls, %u sweep calls)\n",
        ok ? "PASS" : "FAIL", errors.errors, tireCalls, sweepCalls);
    return ok ? 0 : 3;
}
