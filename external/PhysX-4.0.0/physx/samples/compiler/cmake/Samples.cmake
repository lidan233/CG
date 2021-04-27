##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2018 NVIDIA Corporation. All rights reserved.

#
# Build Samples common
#

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/samples/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/Samples.cmake)

SET(SAMPLEBRIDGES_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplebridges)
SET(SAMPLEBRIDGES_SOURCE_FILES
	${SAMPLEBRIDGES_SOURCE_DIR}/SampleBridges.cpp
	${SAMPLEBRIDGES_SOURCE_DIR}/SampleBridges.h
	${SAMPLEBRIDGES_SOURCE_DIR}/SampleBridgesCCT.cpp
	${SAMPLEBRIDGES_SOURCE_DIR}/SampleBridgesInputEventIds.h
	${SAMPLEBRIDGES_SOURCE_DIR}/SampleBridgesSettings.h
)
SOURCE_GROUP("Sample bridges" FILES ${SAMPLEBRIDGES_SOURCE_FILES})

SET(SAMPLEHELLOWORLD_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplehelloworld)
SET(SAMPLEHELLOWORLD_SOURCE_FILES
	${SAMPLEHELLOWORLD_SOURCE_DIR}/SampleHelloWorld.h
	${SAMPLEHELLOWORLD_SOURCE_DIR}/SampleHelloWorld.cpp
)
SOURCE_GROUP("Sample helloworld" FILES ${SAMPLEHELLOWORLD_SOURCE_FILES})

SET(SAMPLELARGEWORLD_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplelargeworld)
SET(SAMPLELARGEWORLD_SOURCE_FILES
	${SAMPLELARGEWORLD_SOURCE_DIR}/ChunkLoader.cpp
	${SAMPLELARGEWORLD_SOURCE_DIR}/ChunkLoader.h
	${SAMPLELARGEWORLD_SOURCE_DIR}/MeshBuilder.cpp
	${SAMPLELARGEWORLD_SOURCE_DIR}/MeshBuilder.h
	${SAMPLELARGEWORLD_SOURCE_DIR}/SampleLargeWorld.cpp
	${SAMPLELARGEWORLD_SOURCE_DIR}/SampleLargeWorld.h
	${SAMPLELARGEWORLD_SOURCE_DIR}/SampleLargeWorldCCT.cpp
	${SAMPLELARGEWORLD_SOURCE_DIR}/SampleLargeWorldInputEventIds.h	
)
SOURCE_GROUP("Sample large world" FILES ${SAMPLELARGEWORLD_SOURCE_FILES})

SET(SAMPLENORTHPOLE_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplenorthpole)
SET(SAMPLENORTHPOLE_SOURCE_FILES 
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPole.cpp
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPole.h
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleBuilder.cpp
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleCameraController.cpp
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleCameraController.h
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleCCT.cpp
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleDynamics.cpp
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleFilterShader.cpp
	${SAMPLENORTHPOLE_SOURCE_DIR}/SampleNorthPoleInputEventIds.h
)
SOURCE_GROUP("Sample north pole" FILES ${SAMPLENORTHPOLE_SOURCE_FILES})

SET(SAMPLESUBMARINE_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplesubmarine)
SET(SAMPLESUBMARINE_SOURCE_FILES 
	${SAMPLESUBMARINE_SOURCE_DIR}/Crab.cpp
	${SAMPLESUBMARINE_SOURCE_DIR}/Crab.h
	${SAMPLESUBMARINE_SOURCE_DIR}/SampleSubmarine.cpp
	${SAMPLESUBMARINE_SOURCE_DIR}/SampleSubmarine.h
	${SAMPLESUBMARINE_SOURCE_DIR}/SampleSubmarineInputEventIds.h
	${SAMPLESUBMARINE_SOURCE_DIR}/SubmarineCameraController.h
)
SOURCE_GROUP("Sample submarine" FILES ${SAMPLESUBMARINE_SOURCE_FILES})

SET(SAMPLEVEHICLE_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplevehicle)
SET(SAMPLEVEHICLE_SOURCE_FILES 
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicleDebugRender.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicleInputEventIds.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicleTerrain.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_CameraController.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_CameraController.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_ControlInputs.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_ControlInputs.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_GameLogic.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_GameLogic.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_SceneQuery.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_SceneQuery.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_VehicleController.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_VehicleController.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_VehicleCooking.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_VehicleCooking.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_VehicleManager.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_VehicleManager.h
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_WheelQueryResults.cpp
	${SAMPLEVEHICLE_SOURCE_DIR}/SampleVehicle_WheelQueryResults.h
)
SOURCE_GROUP("Sample vehicle" FILES ${SAMPLEVEHICLE_SOURCE_FILES})

SET(SAMPLECUSTOMGRAVITY_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplecustomgravity)
SET(SAMPLECUSTOMGRAVITY_SOURCE_FILES 
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/Planet.cpp
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/Planet.h
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravity.cpp
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravity.h
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravity_CCT.cpp
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravityBuilder.cpp
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravityCameraController.cpp
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravityCameraController.h	
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravityFilterShader.cpp	
	${SAMPLECUSTOMGRAVITY_SOURCE_DIR}/SampleCustomGravityInputEventIds.h	
)
SOURCE_GROUP("Sample custom gravity" FILES ${SAMPLECUSTOMGRAVITY_SOURCE_FILES})

SET(SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR ${PHYSX_ROOT_DIR}/samples/samplecctsharedcode)
SET(SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_FILES 
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/KinematicPlatform.cpp
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/KinematicPlatform.h
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/SampleCCTActor.cpp
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/SampleCCTActor.h
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/SampleCCTCameraController.cpp
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/SampleCCTCameraController.h
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/SampleCCTJump.cpp
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_DIR}/SampleCCTJump.h
)
SOURCE_GROUP("Sample CCT shared" FILES ${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_FILES})

ADD_EXECUTABLE(Samples 	
	${SAMPLEBRIDGES_SOURCE_FILES}	
	${SAMPLECUSTOMGRAVITY_SOURCE_FILES}
	${SAMPLEHELLOWORLD_SOURCE_FILES}
	${SAMPLELARGEWORLD_SOURCE_FILES}
	${SAMPLENORTHPOLE_SOURCE_FILES}
	${SAMPLESUBMARINE_SOURCE_FILES}
	${SAMPLEVEHICLE_SOURCE_FILES}
	${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_FILES}	
)

SOURCE_GROUP(SampleBridges FILES ${SAMPLEBRIDGES_SOURCE_FILES})
SOURCE_GROUP(SampleCustomGravity FILES ${SAMPLECUSTOMGRAVITY_SOURCE_FILES})
SOURCE_GROUP(SampleHelloWorld FILES ${SAMPLEHELLOWORLD_SOURCE_FILES})
SOURCE_GROUP(SampleLargeWorld FILES ${SAMPLELARGEWORLD_SOURCE_FILES})
SOURCE_GROUP(SampleNorthPole FILES ${SAMPLENORTHPOLE_SOURCE_FILES})
SOURCE_GROUP(SampleSubmarine FILES ${SAMPLESUBMARINE_SOURCE_FILES})
SOURCE_GROUP(SampleVehicle FILES ${SAMPLEVEHICLE_SOURCE_FILES})
SOURCE_GROUP(CCTShared FILES ${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_FILES})

TARGET_INCLUDE_DIRECTORIES(Samples
	
	PRIVATE ${PHYSX_ROOT_DIR}/include/
	
	PRIVATE ${PHYSX_ROOT_DIR}/samples/samplecctsharedcode
	PRIVATE ${PHYSX_ROOT_DIR}/samples/samplebase
	PRIVATE ${PHYSX_ROOT_DIR}/samples/sampleframework/framework/include
	PRIVATE ${PHYSX_ROOT_DIR}/samples/sampleframework/renderer/include
	PRIVATE ${PHYSX_ROOT_DIR}/samples/sampleframework/platform/include

	PRIVATE ${PHYSX_ROOT_DIR}/source/common/include
	PRIVATE ${PHYSX_ROOT_DIR}/source/common/src
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/include
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/include
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/contact
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/common
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/convex
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/distance
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/gjk
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/intersection
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/mesh
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/hf
	PRIVATE ${PHYSX_ROOT_DIR}/source/geomutils/src/pcm
	PRIVATE ${PHYSX_ROOT_DIR}/source/PhysXVehicle/src
)

TARGET_COMPILE_DEFINITIONS(Samples
	PRIVATE ${SAMPLES_COMPILE_DEFS}
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS)
	SET_TARGET_PROPERTIES(Samples PROPERTIES 
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX_EXE_OUTPUT_DIRECTORY_DEBUG}
		RUNTIME_OUTPUT_DIRECTORY_PROFILE ${PX_EXE_OUTPUT_DIRECTORY_PROFILE}
		RUNTIME_OUTPUT_DIRECTORY_CHECKED ${PX_EXE_OUTPUT_DIRECTORY_CHECKED}
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX_EXE_OUTPUT_DIRECTORY_RELEASE}

		OUTPUT_NAME Samples${EXE_SUFFIX}
	)
ELSE()
	IF(APPEND_CONFIG_NAME)
		SET_TARGET_PROPERTIES(Samples PROPERTIES
			DEBUG_OUTPUT_NAME SamplesDEBUG
			PROFILE_OUTPUT_NAME SamplesPROFILE
			CHECKED_OUTPUT_NAME SamplesCHECKED
			RELEASE_OUTPUT_NAME Samples
		)
	ENDIF()
ENDIF()

TARGET_LINK_LIBRARIES(Samples 
	PUBLIC SampleBase SamplePlatform SampleFramework SampleRenderer SampleToolkit PhysX PhysXExtensions PhysXCharacterKinematic PhysXCooking PhysXFoundation PhysXVehicle SampleToolkit
	PUBLIC ${SAMPLES_PLATFORM_LINKED_LIBS}
)

IF(PX_GENERATE_SOURCE_DISTRO)	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLEBRIDGES_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLECUSTOMGRAVITY_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLEHELLOWORLD_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLELARGEWORLD_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLENORTHPOLE_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLESUBMARINE_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLEVEHICLE_SOURCE_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SAMPLECHARACTERCONTROLLER_SHARED_SOURCE_FILES})
ENDIF()
