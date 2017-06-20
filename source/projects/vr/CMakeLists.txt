cmake_minimum_required(VERSION 3.0)

find_package(OpenGL)

if (WIN32)
	# Windows
	if (CMAKE_SIZEOF_VOID_P EQUAL 8)
		# Windows 64-bit
		SET(OpenVR_LIB ${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/win64/openvr_api.lib)
		SET(Oculus_LIB ${CMAKE_CURRENT_SOURCE_DIR}/LibOVR/Lib/Windows/Win32/Release/VS2015/LibOVR.lib)
	else ()
		# Windows 32-bit
		SET(OpenVR_LIB ${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/win32/openvr_api.lib)
		SET(Oculus_LIB ${CMAKE_CURRENT_SOURCE_DIR}/LibOVR/Lib/Windows/Win32/Release/VS2015/LibOVR.lib)
	endif ()
else ()
	
	# OSX -- no libs right now
	
endif ()

include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-api/script/max-pretarget.cmake)


include_directories( 
	"${CMAKE_CURRENT_SOURCE_DIR}/../../"
	"${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers/"
	"${CMAKE_CURRENT_SOURCE_DIR}/LibOVR/Include/"
	"${C74_INCLUDES}"

)


add_library( 
	${PROJECT_NAME} 
	MODULE
	${PROJECT_NAME}.cpp
)

target_link_libraries(
   ${PROJECT_NAME}
   ${OPENGL_gl_LIBRARY}
   ${OpenVR_LIB}
   ${Oculus_LIB}
)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-api/script/max-posttarget.cmake)
