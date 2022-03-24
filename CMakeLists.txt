# To build, run the following commands in current directory:
# cmake -DVENDOR=cypress -DBOARD=CY8CKIT_064S0S2_4343W -DCOMPILER=arm-gcc -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=../../../tools/cmake/toolchains/arm-gcc.cmake
# cmake --build build

cmake_minimum_required(VERSION 3.13)

# Configure compiler assembler for code outside amazon-freertos source tree
enable_language(C)
enable_language(ASM)

set(afr_app_name "afr-example-mqtt-ml-gesture-classification")
project(${afr_app_name})

set(AFR_PATH "${CMAKE_SOURCE_DIR}/../../..")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(ML_MODEL_NAME "MAGIC_WAND")
set(VFP_SELECT "HARDFP")

# ML library configuration
# Options include:
# - COMPONENT_ML_FLOAT32
# - COMPONENT_ML_INT8x8
# - COMPONENT_ML_INT8x16
# - COMPONENT_ML_INT16x8
# - COMPONENT_ML_INT16x16
set(ML_LIB_SELECT "COMPONENT_ML_FLOAT32")

string(REPLACE "_" "-" APP_BOARD_NAME "${BOARD}")

# Neural Network Configuration. Options include
#
# float    -- floating point for the input data and weights
# int8x8   -- 8-bit fixed-point for the input data and weights
# int16x8  -- 16-bit fixed-point for the input data and 8-bit for weights
# int16x16 -- 16-bit fixed-point for the input data and weights
set(NN_TYPE "float")
set(MODEL_NAME ${ML_MODEL_NAME})

if("${NN_TYPE}" STREQUAL "float")
    set(NN_TYPE_VALUE "CY_ML_FLOATING_POINT_fltxflt_NN=1")
    set(NN_TYPE_COMPONENT "ML_FLOAT32")
elseif("${NN_TYPE}" STREQUAL "int8x8")
    set(NN_TYPE_VALUE "CY_ML_FIXED_POINT_16_IN=1 CY_ML_FIXED_POINT_16_NN=1")
    set(NN_TYPE_COMPONENT "ML_INT16x16")
elseif("${NN_TYPE}" STREQUAL "int16x8")
    set(NN_TYPE_VALUE "CY_ML_FIXED_POINT_16_IN=1 CY_ML_FIXED_POINT_8_NN=1")
    set(NN_TYPE_COMPONENT "ML_INT16x8")
    elseif("${NN_TYPE}" STREQUAL "int16x16")
    set(NN_TYPE_VALUE "CY_ML_FIXED_POINT_16_IN=1 CY_ML_FIXED_POINT_8_NN=1")
    set(NN_TYPE_COMPONENT "ML_INT8x8")
endif()

set(NN_MODEL_NAME ${ML_MODEL_NAME})
                        
add_definitions(-DCY_RETARGET_IO_CONVERT_LF_TO_CRLF
                -D${NN_TYPE_VALUE}
                -DNN_MODEL_NAME=${ML_MODEL_NAME}
                -DMODEL_NAME=${NN_MODEL_NAME}
                -DCOMPONENT_${VFP_SELECT}
                -DCY_BMI_160_IMU=1
                -DCOMPONENT_${NN_TYPE_COMPONENT}
                -D${NN_TYPE_COMPONENT})

# Path to this application directory
get_filename_component(CY_APP_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
set(AFR_BOARD "${VENDOR}.${BOARD}" CACHE INTERNAL "")
set(AFR_TOOLCHAIN "${COMPILER}" CACHE INTERNAL "")

# Set ML library extension based on compiler chosen
if("${AFR_TOOLCHAIN}" STREQUAL "arm-gcc")
    set(LIB_EXTN "a")
    set(TOOLCHAIN GCC_ARM)
elseif("${AFR_TOOLCHAIN}" STREQUAL "arm-armclang")
    set(LIB_EXTN "ar")
    set(TOOLCHAIN ARM)
elseif("${AFR_TOOLCHAIN}" STREQUAL "arm-iar")
    set(LIB_EXTN "a")
    set(TOOLCHAIN IAR)
endif()

# Configure include for freertos config files, aws config files, and cypress psoc6_core header files before add_subdirectory
set(BOARD_DEMO_DIR "${AFR_PATH}/vendors/cypress/boards/${BOARD}/aws_demos")
set(BOARD_CY_CODE_DIR "${BOARD_DEMO_DIR}/application_code/cy_code")
set(additional_include_dirs "${BOARD_DEMO_DIR}/config_files"
                            "${BOARD_CY_CODE_DIR}"
                            "${BOARD_CY_CODE_DIR}/GeneratedSource"
                            "${AFR_PATH}/demos/include"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/source"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/fifo"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/source/ml-inference/include"
							"${AFR_PATH}/projects/cypress/${afr_app_name}/source/ml-middleware/"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/source/sensor-motion-bmi160"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/mtb_ml_gen/mtb_ml_models"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/mtb_ml_gen/mtb_ml_regression_data"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/source/ml-inference/${ML_LIB_SELECT}/COMPONENT_${VFP_SELECT}/TOOLCHAIN_${TOOLCHAIN}"
                            "${AFR_PATH}/projects/cypress/${afr_app_name}/bmi160"
                            "${AFR_PATH}/libraries/abstractions/platform/include/platform"
                            "${AFR_PATH}/demos/common/pkcs11_helpers"
                            "${AFR_PATH}/libraries/3rdparty/unity/src"
                            "${AFR_PATH}/libraries/3rdparty/unity/extras/fixture/src"
                            "${AFR_PATH}/libraries/c_sdk/standard/common/include"
                            "${AFR_PATH}/libraries/coreMQTT/source/include"
                            "${AFR_PATH}/vendors/cypress/MTB/port_support/kvstore"
                            "${AFR_PATH}/libraries//abstractions/transport/secure_sockets"
                            "${AFR_PATH}/libraries/abstractions/backoff_algorithm/source/include"
                            "${AFR_PATH}/libraries/abstractions/backoff_algorithm/source/include"
                            "${AFR_PATH}/vendors/cypress/MTB/libraries/wifi-host-driver/WiFi_Host_Driver/resources/nvram/TARGET_CY8CKIT_064B0S2_4343W"
                            "${AFR_PATH}/libraries/coreMQTT/source/interface")

if (EXISTS "${BOARD_CY_CODE_DIR}/SDIO_HOST")
    list(APPEND additional_include_dirs "${BOARD_CY_CODE_DIR}/SDIO_HOST")
endif()

set(CY_INCLUDE_DIRS ${additional_include_dirs} CACHE INTERNAL "")

# build amazon-freertos as library
set(LIB_MODE 1)

# build amazon-freertos
add_subdirectory(${AFR_PATH} "${AFR_PATH}/build")

# add executable target source files
add_executable(${afr_app_name} "${CMAKE_SOURCE_DIR}/main.c"
                               "${CMAKE_SOURCE_DIR}/source/control.c"
                               "${CMAKE_SOURCE_DIR}/source/iot_demo_mqtt_ml_gesture_classifier.c"
                               "${CMAKE_SOURCE_DIR}/source/mqtt_operation.c"
                               "${CMAKE_SOURCE_DIR}/source/processing.c"
                               "${CMAKE_SOURCE_DIR}/source/gesture.c"
                               "${CMAKE_SOURCE_DIR}/fifo/cy_fifo.c"
							   "${CMAKE_SOURCE_DIR}/source/ml-middleware/mtb_ml_model.c"
							   "${CMAKE_SOURCE_DIR}/source/ml-middleware/mtb_ml_utils.c"
                               "${CMAKE_SOURCE_DIR}/bmi160/bmi160.c"
                               "${CMAKE_SOURCE_DIR}/source/cJSON/cJSON.c"
                               "${CMAKE_SOURCE_DIR}/source/BMM150-Sensor-API/bmm150.c"
                               "${CMAKE_SOURCE_DIR}/source/sensor-motion-bmi160/mtb_bmi160.c")


include("${AFR_PATH}/vendors/cypress/MTB/psoc6/cmake/cy_defines.cmake")
include("${AFR_PATH}/vendors/cypress/MTB/psoc6/cmake/cy_create_exe_target.cmake")
include("${AFR_PATH}/vendors/cypress/MTB/psoc6/cmake/cy_create_linked_target.cmake")

# Add board specific files (taken from amazon-freertos/vendors/cypress/boards/${BOARD}/aws_demos/application_code/cy_code).
# Customize as necessary or replace with another cypress board.
# The code in ${BSP_DIR}/GeneratedSource is generated by the Configurator tools while saving ${BSP_DIR}/design.modus.
# Exclude the BSP design.modus when CUSTOM_DESIGN_MODUS is defined.
cy_find_files(exe_glob_files DIRECTORY "${BOARD_CY_CODE_DIR}")
if (DEFINED CUSTOM_DESIGN_MODUS)
    foreach(item ${exe_glob_files})
        if(${item} MATCHES "GeneratedSource")
            list(REMOVE_ITEM exe_glob_files ${item})
        endif(${item} MATCHES "GeneratedSource")
    endforeach()
endif()
cy_get_src(exe_source_files ITEMS "${exe_glob_files}")
target_sources(${afr_app_name} PUBLIC "${exe_source_files}")

# Application extra sources
file(GLOB_RECURSE CY_APP_EXTRA_SOURCES "${AFR_PATH}/libraries/c_sdk/standard/common/iot_device_metrics.c"
                                       "${AFR_PATH}/libraries/coreMQTT/source/*.c"
                                       "${AFR_PATH}/demos/common/pkcs11_helpers/pkcs11_helpers.c"
                                       "${AFR_PATH}/libraries/abstractions/transport/secure_sockets/transport_secure_sockets.c"
                                       "${AFR_PATH}/libraries/logging/*.c"
                                       "${AFR_PATH}/vendors/cypress/MTB/port_support/kvstore/kvstore.c"
                                       "${AFR_PATH}/libraries/abstractions/backoff_algorithm/source/backoff_algorithm.c"
                                       "${AFR_PATH}/demos/mqtt/*.c")

target_sources(${afr_app_name} PUBLIC "${CY_APP_EXTRA_SOURCES}")

# add linker script and map file generation
cy_add_linker_options(EXE_APP_NAME ${afr_app_name})

# Set up dependent linked libraries
target_link_libraries(${afr_app_name}  PUBLIC
    AFR::demo_base
    AFR::common
    AFR::utils
    AFR::wifi
    AFR::wifi::mcu_port
    psoc6_core # Pulls in defines, includes, source code for psoc6 devices
    ${CMAKE_SOURCE_DIR}/source/ml-inference/${ML_LIB_SELECT}/COMPONENT_${VFP_SELECT}/TOOLCHAIN_${TOOLCHAIN}/libml-inference.${LIB_EXTN}
)
