# --- Application name
set(ROBOTOPT_NAME ${SOLUTION_NAME})

# --- Gather sources
file(GLOB ROBOTOPT_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
file(GLOB ROBOTOPT_INCS ${CMAKE_CURRENT_LIST_DIR}/src/*.h)

# Sparese/Dense Matrices
file(GLOB ROBOTOPT_INC_DENSE ${NATID_SDK_INC}/dense/*.h)
file(GLOB ROBOTOPT_INC_DENSE_PRIV ${NATID_SDK_INC}/dense/priv/*.h)
file(GLOB ROBOTOPT_INC_SPARSE ${NATID_SDK_INC}/sparse/*.h)
file(GLOB ROBOTOPT_INC_SPARSE_PRIV ${NATID_SDK_INC}/sparse/priv/*.h)

# Math and Matrix utilities
file(GLOB ROBOTOPT_INC_MATH ${NATID_SDK_INC}/math/*.h)
file(GLOB ROBOTOPT_INC_MATRIX ${NATID_SDK_INC}/matrix/*.h)
file(GLOB ROBOTOPT_INC_MTX ${NATID_SDK_INC}/mtx/*.h)
file(GLOB ROBOTOPT_INC_MU ${NATID_SDK_INC}/mu/*.h)
file(GLOB ROBOTOPT_INC_MEM ${NATID_SDK_INC}/mem/*.h)

# GUI and other utilities
file(GLOB ROBOTOPT_INC_TD ${NATID_SDK_INC}/td/*.h)
file(GLOB ROBOTOPT_INC_GUI ${NATID_SDK_INC}/gui/*.h)

file(GLOB ROBOTOPT_INC_THREAD  ${NATID_SDK_INC}/thread/*.h)
file(GLOB ROBOTOPT_INC_CNT  ${NATID_SDK_INC}/cnt/*.h)
file(GLOB ROBOTOPT_INC_FO  ${NATID_SDK_INC}/fo/*.h)
file(GLOB ROBOTOPT_INC_XML  ${NATID_SDK_INC}/xml/*.h)

# --- Application icon
set(ROBOTOPT_PLIST  ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/AppIcon.plist)
if(WIN32)
	set(ROBOTOPT_WINAPP_ICON ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/winAppIcon.rc)
else()
	set(ROBOTOPT_WINAPP_ICON ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/winAppIcon.cpp)
endif()

add_executable(${ROBOTOPT_NAME}
    ${ROBOTOPT_INCS}
    ${ROBOTOPT_SOURCES}
    ${ROBOTOPT_INC_DENSE}
    ${ROBOTOPT_INC_DENSE_PRIV}
    ${ROBOTOPT_INC_SPARSE}
    ${ROBOTOPT_INC_SPARSE_PRIV}
    ${ROBOTOPT_INC_MATH}
    ${ROBOTOPT_INC_MATRIX}
    ${ROBOTOPT_INC_MTX}
    ${ROBOTOPT_INC_MU}
    ${ROBOTOPT_INC_MEM}
    ${ROBOTOPT_INC_TD}
    ${ROBOTOPT_INC_GUI}
    ${ROBOTOPT_INC_FO}
    ${ROBOTOPT_INC_CNT}
    ${ROBOTOPT_INC_XML}
    ${ROBOTOPT_INC_THREAD}
    ${ROBOTOPT_WINAPP_ICON}
)

source_group("src" FILES ${ROBOTOPT_SOURCES})
source_group("inc" FILES ${ROBOTOPT_INCS})
source_group("inc\\dense" FILES ${ROBOTOPT_INC_DENSE})
source_group("inc\\dense\\priv" FILES ${ROBOTOPT_INC_DENSE_PRIV})
source_group("inc\\sparse" FILES ${ROBOTOPT_INC_SPARSE})
source_group("inc\\sparse\\priv" FILES ${ROBOTOPT_INC_SPARSE_PRIV})
source_group("inc\\fo" FILES ${ROBOTOPT_INC_FO})
source_group("inc\\gui" FILES ${ROBOTOPT_INC_GUI})
source_group("inc\\math" FILES ${ROBOTOPT_INC_MATH})
source_group("inc\\matrix" FILES ${ROBOTOPT_INC_MATRIX})
source_group("inc\\mtx" FILES ${ROBOTOPT_INC_MTX})
source_group("inc\\mu" FILES ${ROBOTOPT_INC_MU})
source_group("inc\\mem" FILES ${ROBOTOPT_INC_MEM})
source_group("inc\\td" FILES ${ROBOTOPT_INC_TD})
source_group("inc\\cnt" FILES ${ROBOTOPT_INC_CNT})
source_group("inc\\thread" FILES ${ROBOTOPT_INC_THREAD})
source_group("inc\\xml" FILES ${ROBOTOPT_INC_XML})

target_compile_definitions(${ROBOTOPT_NAME} PUBLIC MU_USETIMER)

target_link_libraries(${ROBOTOPT_NAME}
    debug ${MU_LIB_DEBUG}
    debug ${MATRIX_LIB_DEBUG}
    debug ${NATGUI_LIB_DEBUG}  
    optimized ${MU_LIB_RELEASE}
    optimized ${MATRIX_LIB_RELEASE}
    optimized ${NATGUI_LIB_RELEASE}
)

# --- Apply macros
setIDEPropertiesForExecutable(${ROBOTOPT_NAME} ${CMAKE_CURRENT_LIST_DIR})
setAppIcon(${ROBOTOPT_NAME} ${CMAKE_CURRENT_LIST_DIR})
setIDEPropertiesForGUIExecutable(${ROBOTOPT_NAME} ${CMAKE_CURRENT_LIST_DIR})
setTargetPropertiesForGUIApp(${ROBOTOPT_NAME} ${ROBOTOPT_PLIST})
set_target_properties(${ROBOTOPT_NAME} PROPERTIES
    VS_DEBUGGER_COMMAND_ARGUMENTS "-devResPath=${CMAKE_CURRENT_LIST_DIR}")
setPlatformDLLPath(${ROBOTOPT_NAME})

# --- Linux icon installation
if(UNIX AND NOT APPLE)
    set(ICON_SIZES 16 32 48 128 256)
    foreach(SIZE ${ICON_SIZES})
        install(FILES ${CMAKE_CURRENT_LIST_DIR}/res/appIcon/lnxApp${SIZE}.png
                DESTINATION share/icons/hicolor/${SIZE}x${SIZE}/apps
                RENAME robottrajectoryopt.png)
    endforeach()
    
    # Install .desktop file
    install(FILES ${CMAKE_CURRENT_LIST_DIR}/RobotTrajectoryOpt.desktop
            DESTINATION share/applications)
endif()
