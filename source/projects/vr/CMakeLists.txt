cmake_minimum_required(VERSION 3.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-sdk/script/max-pretarget.cmake)

#############################################################
# MAX EXTERNAL
#############################################################

include_directories( 
	"${MAX_SDK_INCLUDES}"
	"${MAX_SDK_MSP_INCLUDES}"
	"${MAX_SDK_JIT_INCLUDES}"
	"${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers"
	"${CMAKE_CURRENT_SOURCE_DIR}/LibOVR/Include"
	"${CMAKE_CURRENT_SOURCE_DIR}/../.."
)

add_library( 
	${PROJECT_NAME} 
	MODULE
	vr.cpp
	${MAX_SDK_INCLUDES}/common/commonsyms.c
	"${CMAKE_CURRENT_SOURCE_DIR}/../../al_math.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/../../al_max.h"
)

if (APPLE)

else ()
	find_package(OpenGL REQUIRED)
	include_directories(${OPENGL_INCLUDE_DIR})
	target_link_libraries(${PROJECT_NAME} PUBLIC ${OPENGL_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} PUBLIC 
		"${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/win64/openvr_api.lib"
		"${CMAKE_CURRENT_SOURCE_DIR}/LibOVR/Lib/Windows/x64/Release/VS2015/LibOVR.lib"
	)	
endif ()


include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-sdk/script/max-posttarget.cmake)
