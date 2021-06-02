cmake_minimum_required(VERSION 3.6)

project(rak7200-tracker)

set(LORAMAC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/LoRaMac-node)

set(SECURE_ELEMENT SOFT_SE)
set(RADIO sx1276)
set(BOARD rak7200)
set(ACTIVE_REGION LORAMAC_REGION_RU864)
set(LINKER_SCRIPT ${LORAMAC_DIR}/src/boards/NucleoL073/cmsis/arm-gcc/stm32l073xx_flash.ld)
include(stm32l0)

add_subdirectory(${LORAMAC_DIR}/src/boards)
add_subdirectory(${LORAMAC_DIR}/src/radio)
add_subdirectory(${LORAMAC_DIR}/src/system)
add_subdirectory(${LORAMAC_DIR}/src/mac)
add_subdirectory(${LORAMAC_DIR}/src/peripherals)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/board)

list(APPEND ${PROJECT_NAME}_COMMON
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/CayenneLpp.c"
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/NvmDataMgmt.c"
        )

list(APPEND ${PROJECT_NAME}_LMH
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/LmHandler.c"
        )

list(APPEND ${PROJECT_NAME}_LMHP
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/packages/FragDecoder.c"
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/packages/LmhpClockSync.c"
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/packages/LmhpCompliance.c"
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/packages/LmhpFragmentation.c"
        "${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/packages/LmhpRemoteMcastSetup.c"
        )

set(${PROJECT_NAME}_SOURCES src/main.c)

add_executable(${PROJECT_NAME}
        ${${PROJECT_NAME}_COMMON}
        ${${PROJECT_NAME}_LMH}
        ${${PROJECT_NAME}_LMHP}
        ${${PROJECT_NAME}_SOURCES}
        $<TARGET_OBJECTS:mac>
        $<TARGET_OBJECTS:system>
        $<TARGET_OBJECTS:radio>
        $<TARGET_OBJECTS:peripherals>
        $<TARGET_OBJECTS:${BOARD}>
        )

target_link_libraries(${PROJECT_NAME} m)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_HAL_DRIVER -DSTM32L073xx ACTIVE_REGION=${ACTIVE_REGION})
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${LORAMAC_DIR}/src/boards
        ${LORAMAC_DIR}/src/apps/LoRaMac/common
        ${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler
        ${LORAMAC_DIR}/src/apps/LoRaMac/common/LmHandler/packages
        $<BUILD_INTERFACE:$<TARGET_PROPERTY:mac,INTERFACE_INCLUDE_DIRECTORIES>>
        $<BUILD_INTERFACE:$<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>>
        $<BUILD_INTERFACE:$<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>>
        $<BUILD_INTERFACE:$<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>>
        )

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DUSE_FULL_ASSERT)
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)
