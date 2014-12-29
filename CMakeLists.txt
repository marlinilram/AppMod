cmake_minimum_required( VERSION 2.8 )
project( ImageToShape )

SET( CMAKE_PREFIX_PATH "D:\\Qt\\Qt5.3.2\\5.3\\msvc2013_64_opengl" )

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

find_package( Qt5Widgets )
find_package( Qt5Xml )
find_package( Qt5OpenGL )
find_package( Qt5Core )

FIND_PACKAGE( OpenGL REQUIRED )
FIND_PACKAGE( GLEW REQUIRED )
find_package( OpenCV REQUIRED )

FILE( GLOB Project_SRCS "src/*.cpp"
						"src/ui/*.cpp" 
						"src/Model/*.cpp"
						"src/OBJ/*.cc" 
						"src/LOG/*.cpp"
						"src/Samples/*.cpp"
						"src/Light/*.cpp"
						"src/Ray/*.cpp"
						"src/Viewer/*.cpp" 
						"src/Camera/*.cpp"
						"src/Alg/*.cpp"
						"src/Deform/*.cpp" )
						
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
						 src/extern 
						 src/extern/eigen )

SET( MainWin_UIS src/ui/mainWin.ui )
SET( MainWin_MOC_HDRS src/ui/mainWin.h 
					  src/Alg/ImagePartAlg.h )
						 
add_definitions(-DNOMINMAX)
add_definitions(-D_USE_MATH_DEFINES)

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

ADD_EXECUTABLE( ${PROJECT_NAME} 
				${Project_SRCS}  
				${MainWin_UIS_H} 
				${MainWin_MOC_SRCS} )
				
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} 
					   ${Qt5Widgets_LIBRARIES} 
					   ${Qt5Xml_LIBRARIES}
					   ${Qt5OpenGL_LIBRARIES}
					   ${OPENGL_LIBRARIES}
					   ${OpenCV_LIBS}
					   ${GLEW_LIBRARIES}
					   debug ${QGLViewer_lib_debug} 
					   optimized ${QGLViewer_lib_release} )
					   
MESSAGE( STATUS "${OPENGL_LIBRARIES}" )