set(MPCCORE_NAME ${SOLUTION_NAME})

file(GLOB MPCCORE_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
file(GLOB MPCCORE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/src/*.h)
file(GLOB MPCCORE_INC_DENSE ${NATID_SDK_INC}/dense/*.h)
file(GLOB MPCCORE_INC_DENSE_PRIV ${NATID_SDK_INC}/dense/priv/*.h)
file(GLOB MPCCORE_INC_SPARSE ${NATID_SDK_INC}/sparse/*.h)
file(GLOB MPCCORE_INC_SPARSE_PRIV ${NATID_SDK_INC}/sparse/priv/*.h)
file(GLOB MPCCORE_INC_FO ${NATID_SDK_INC}/fo/*.h)
file(GLOB MPCCORE_INC_GUI ${NATID_SDK_INC}/gui/*.h)
file(GLOB MPCCORE_INC_MATH ${NATID_SDK_INC}/math/*.h)
file(GLOB MPCCORE_INC_MATRIX ${NATID_SDK_INC}/matrix/*.h)
file(GLOB MPCCORE_INC_MTX ${NATID_SDK_INC}/mtx/*.h)
file(GLOB MPCCORE_INC_MU ${NATID_SDK_INC}/mu/*.h)
file(GLOB MPCCORE_INC_MEM ${NATID_SDK_INC}/mem/*.h)
file(GLOB MPCCORE_INC_TD ${NATID_SDK_INC}/td/*.h)

add_executable(${MPCCORE_NAME}
    ${MPCCORE_HEADERS}
    ${MPCCORE_SOURCES}
    ${MPCCORE_INC_DENSE}
    ${MPCCORE_INC_DENSE_PRIV}
    ${MPCCORE_INC_SPARSE}
    ${MPCCORE_INC_SPARSE_PRIV}
    ${MPCCORE_INC_FO}
    ${MPCCORE_INC_GUI}
    ${MPCCORE_INC_MATH}
    ${MPCCORE_INC_MATRIX}
    ${MPCCORE_INC_MTX}
    ${MPCCORE_INC_MU}
    ${MPCCORE_INC_MEM}
    ${MPCCORE_INC_TD}
)

source_group("src" FILES ${MPCCORE_SOURCES})
source_group("inc" FILES ${MPCCORE_HEADERS})
source_group("inc\\dense" FILES ${MPCCORE_INC_DENSE})
source_group("inc\\dense\\priv" FILES ${MPCCORE_INC_DENSE_PRIV})
source_group("inc\\sparse" FILES ${MPCCORE_INC_SPARSE})
source_group("inc\\sparse\\priv" FILES ${MPCCORE_INC_SPARSE_PRIV})
source_group("inc\\fo" FILES ${MPCCORE_INC_FO})
source_group("inc\\gui" FILES ${MPCCORE_INC_GUI})
source_group("inc\\math" FILES ${MPCCORE_INC_MATH})
source_group("inc\\matrix" FILES ${MPCCORE_INC_MATRIX})
source_group("inc\\mtx" FILES ${MPCCORE_INC_MTX})
source_group("inc\\mu" FILES ${MPCCORE_INC_MU})
source_group("inc\\mem" FILES ${MPCCORE_INC_MEM})
source_group("inc\\td" FILES ${MPCCORE_INC_TD})

target_compile_definitions(${MPCCORE_NAME} PUBLIC MU_USETIMER)

target_link_libraries(${MPCCORE_NAME}
    debug ${MU_LIB_DEBUG} optimized ${MU_LIB_RELEASE}
    debug ${MATRIX_LIB_DEBUG} optimized ${MATRIX_LIB_RELEASE}
)

setIDEPropertiesForExecutable(${MPCCORE_NAME})

