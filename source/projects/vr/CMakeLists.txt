cmake_minimum_required(VERSION 3.0)

find_package(OpenGL)



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
)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-api/script/max-posttarget.cmake)
