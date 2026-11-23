cmake_minimum_required(VERSION 3.21)

add_library(Ev4sioSmokeDll SHARED Smoke.cpp)
target_include_directories(Ev4sioSmokeDll PRIVATE
    ${PHYSX_ROOT_DIR}/include
    ${PHYSX_ROOT_DIR}/source/physxextensions/src
    ${PHYSX_ROOT_DIR}/../blast/include/lowlevel
    ${PHYSX_ROOT_DIR}/../blast/include/extensions/authoringCommon
    ${PHYSX_ROOT_DIR}/../blast/include/shared/NvFoundation)
target_compile_features(Ev4sioSmokeDll PRIVATE cxx_std_14)
target_compile_definitions(Ev4sioSmokeDll PRIVATE PX_PHYSX_STATIC_LIB
    $<$<CONFIG:checked>:PX_CHECKED=1;NDEBUG;_ITERATOR_DEBUG_LEVEL=0>
    $<$<CONFIG:debug>:PX_DEBUG=1;PX_CHECKED=1;_DEBUG>)
target_link_libraries(Ev4sioSmokeDll PRIVATE PhysX PhysXCommon PhysXFoundation
    PhysXExtensions PhysXVehicle PhysXCharacterKinematic PhysXPvdSDK)
if(TARGET PhysXGpu AND PX_GENERATE_GPU_STATIC_LIBRARIES)
    target_compile_definitions(Ev4sioSmokeDll PRIVATE EV4SIO_TEST_GPU PX_PHYSX_GPU_STATIC)
    target_link_libraries(Ev4sioSmokeDll PRIVATE PhysXGpu CUDA::cuda_driver)
    # PhysX implements the CUDA runtime registration/launch entry points in
    # CudaKernelWrangler. CMake otherwise injects cudart_static into consumers
    # of a CUDA static library, producing duplicate definitions.
    set_property(TARGET Ev4sioSmokeDll PROPERTY CUDA_RUNTIME_LIBRARY None)
endif()
set_target_properties(Ev4sioSmokeDll PROPERTIES
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:debug>:Debug>DLL")

add_executable(Ev4sioSmoke Main.cpp)
target_link_libraries(Ev4sioSmoke PRIVATE Ev4sioSmokeDll)
set_target_properties(Ev4sioSmoke PROPERTIES
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:debug>:Debug>DLL")
add_test(NAME Ev4sio.SdkIntegration COMMAND Ev4sioSmoke)

# Compile the adapted VHACD parameter mapping with the same host compiler.
set(BLAST_ROOT ${PHYSX_ROOT_DIR}/../blast)
add_library(Ev4sioBlastAuthoringCheck OBJECT
    ${BLAST_ROOT}/source/sdk/extensions/authoring/NvBlastExtAuthoringCollisionBuilderImpl.cpp)
target_compile_features(Ev4sioBlastAuthoringCheck PRIVATE cxx_std_14)
target_compile_definitions(Ev4sioBlastAuthoringCheck PRIVATE NDEBUG NV_STATIC_LIB)
foreach(dir include/lowlevel include/globals include/extensions/assetutils
    include/extensions/authoring include/extensions/authoringCommon include/shared/NvFoundation
    source/sdk/common source/sdk/extensions/authoring source/sdk/extensions/authoringCommon
    source/sdk/extensions/authoring/VHACD/inc source/sdk/extensions/authoring/VHACD/public
    source/shared/NsFoundation/include)
    target_include_directories(Ev4sioBlastAuthoringCheck PRIVATE ${BLAST_ROOT}/${dir})
endforeach()
add_dependencies(Ev4sioSmoke Ev4sioBlastAuthoringCheck)
