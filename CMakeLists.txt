cmake_minimum_required( VERSION 2.8 )
project( ImageToShape )

SET( CMAKE_PREFIX_PATH "C:\\Qt\\Qt5.2.1\\5.2.1\\msvc2012_64_opengl" 
						"C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64" )

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

find_package( Qt5Widgets )
find_package( Qt5Xml )
find_package( Qt5OpenGL )
find_package( Qt5Core )

find_package( VTK REQUIRED )
include(${VTK_USE_FILE})

FIND_PACKAGE( OpenGL REQUIRED )
FIND_PACKAGE( GLEW REQUIRED )
find_package( OpenCV REQUIRED )

FILE( GLOB Project_SRCS "src/*.*"
						"src/*/*.cpp" 
						"src/*/*.cc")
						
SET( Project_INCLUDE_DIR src/
						 src/ui 
						 src/Model
						 src/OBJ
						 src/LOG
						 src/Samples
						 src/Light
						 src/Ray
						 src/Viewer
						 src/Camera
						 src/Alg
						 src/Deform
						 src/SIFTFlow
						 src/extern
						 src/extern/nlopt/api 
						 src/extern/eigen )

SET( MainWin_UIS src/ui/mainWin.ui )
SET( MainWin_MOC_HDRS src/ui/mainWin.h 
					  src/Alg/ImagePartAlg.h )
						 
add_definitions(-DNOMINMAX)
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

QT5_WRAP_UI( MainWin_UIS_H ${MainWin_UIS} )
QT5_WRAP_CPP( MainWin_MOC_SRCS ${MainWin_MOC_HDRS} )

INCLUDE_DIRECTORIES( ${Project_INCLUDE_DIR} 
					 ${CMAKE_CURRENT_BINARY_DIR} 
					 ${Qt5Widgets_INCLUDE_DIRS} 
					 ${Qt5Xml_INCLUDE_DIRS} 
					 ${Qt5OpenGL_INCLUDE_DIRS} 
					 ${OPENGL_INCLUDE_DIR} )
					 

SET( QGLViewer_lib_debug ${CMAKE_CURRENT_SOURCE_DIR}/lib/QGLViewerd2.lib )
SET( QGLViewer_lib_release ${CMAKE_CURRENT_SOURCE_DIR}/lib/QGLViewer2.lib )
SET( NLopt_lib_debug ${CMAKE_CURRENT_SOURCE_DIR}/lib/nloptd.lib )
SET( NLopt_lib_release ${CMAKE_CURRENT_SOURCE_DIR}/lib/nlopt.lib )

ADD_EXECUTABLE( ${PROJECT_NAME} 
				${Project_SRCS}  
				${MainWin_UIS_H} 
				${MainWin_MOC_SRCS} )
				
				
FILE( GLOB UI 			"src/ui/*.*" ) 
FILE( GLOB Model 		"src/Model/*.*" )
FILE( GLOB TinyOBJ 		"src/OBJ/*.*" )
FILE( GLOB Utility 		"src/LOG/*.*" )
FILE( GLOB Render 		"src/Samples/*.*"
						"src/Light/*.*"
						"src/Ray/*.*" )
FILE( GLOB GLViewer 	"src/Viewer/*.*" 
						"src/Camera/*.*" )
FILE( GLOB SIRDeform 	"src/Alg/*.*"
						"src/Deform/*.*" )
FILE( GLOB SIFTFlow 	"src/SIFTFlow/*.*" )
					 
source_group("UI" 			FILES ${UI} )
source_group("Model" 		FILES ${Model} )
source_group("TinyOBJ" 		FILES ${TinyOBJ} )
source_group("Utility" 		FILES ${Utility} )
source_group("Render" 		FILES ${Render} )
source_group("GLViewer" 	FILES ${GLViewer} )
source_group("Algorithm\\SIRDeform" FILES ${SIRDeform} )
source_group("Algorithm\\SIFTFlow" 	FILES ${SIFTFlow} )
				
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} 
					   ${Qt5Widgets_LIBRARIES} 
					   ${Qt5Xml_LIBRARIES}
					   ${Qt5OpenGL_LIBRARIES}
					   ${OPENGL_LIBRARIES}
					   ${OpenCV_LIBS}
					   ${GLEW_LIBRARIES}
					   ${VTK_LIBRARIES}
					   debug ${NLopt_lib_debug}
					   optimized ${NLopt_lib_release}
					   debug ${QGLViewer_lib_debug} 
					   optimized ${QGLViewer_lib_release} )
					   
MESSAGE( STATUS "${Project_SRCS}" )