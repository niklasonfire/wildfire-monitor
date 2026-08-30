# ADR-0002: the decoding is generated from the Field Table, not written by
# hand. The generation is wired in twice, and both are needed:
#
#   execute_process   runs now, at configure time, because
#                     idf_component_register() below lists the generated .c
#                     among its sources and CMake has to find it on disk while
#                     it is evaluating that call.
#   add_custom_command puts the same generation into the build graph, so that
#                     editing field-table.json regenerates the decoder on the
#                     next build instead of silently linking a stale object.
#
# The generator is Python standard library only, which is what lets it run in
# here as well as on a development machine. Its output lands in the build
# directory and is gitignored: it cannot be edited into and survive.
idf_build_get_property(python PYTHON)

set(WF_TABLE     "${CMAKE_CURRENT_LIST_DIR}/../field-table.json")
set(WF_GENERATOR "${CMAKE_CURRENT_LIST_DIR}/../scripts/gen_fields.py")
set(WF_GEN_DIR   "${CMAKE_BINARY_DIR}/gen")
set(WF_GEN_H     "${WF_GEN_DIR}/wf_fields.h")
set(WF_GEN_C     "${WF_GEN_DIR}/wf_fields.c")

# Both are skipped during IDF's early expansion pass, which runs this file as
# a script only to collect REQUIRES: add_custom_command does not exist there.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    execute_process(
        COMMAND "${python}" "${WF_GENERATOR}" "${WF_TABLE}" --c-dir "${WF_GEN_DIR}"
        RESULT_VARIABLE wf_gen_result
        OUTPUT_VARIABLE wf_gen_output
        ERROR_VARIABLE  wf_gen_output)
    if(NOT wf_gen_result EQUAL 0)
        message(FATAL_ERROR
                "generating the decoder from ${WF_TABLE} failed:\n${wf_gen_output}")
    endif()

    add_custom_command(
        OUTPUT  "${WF_GEN_H}" "${WF_GEN_C}"
        COMMAND "${python}" "${WF_GENERATOR}" "${WF_TABLE}" --c-dir "${WF_GEN_DIR}"
        DEPENDS "${WF_TABLE}" "${WF_GENERATOR}"
        COMMENT "Generating the decoder from the Field Table"
        VERBATIM)
endif()

idf_component_register(
    SRCS "main.c" "board.c" "ble_explorer.c" "cmd_ble.c" "cmd_cap.c"
         "capture.c" "capture_store.c" "display.c" "ui.c" "rtc_bm8563.c"
         "webdump.c" "imu.c" "i2c_bus.c"
         # The NVS adapter for the estimation seam. Firmware, deliberately:
         # main/wfest must not know NVS exists.
         "est_store.c"
         # Pure C99, no ESP-IDF: the same files the host replay harness builds,
         # see the Makefile, tests/host/replay.c and tests/host/unit.c.
         "wfdecode/wfdecode.c" "wfdecode/wfl_read.c"
         "wfest/wfest.c"
         # Generated from field-table.json, above.
         "${WF_GEN_C}"
    INCLUDE_DIRS "." "${WF_GEN_DIR}" "wfdecode" "wfest"
    REQUIRES bt console driver esp_adc esp_driver_gpio esp_driver_i2c
             esp_driver_spi esp_event esp_http_server esp_lcd esp_netif
             esp_timer esp_wifi fatfs nvs_flash spi_flash vfs wear_levelling)

# Determinism, the other half of the Makefile's FPDET. GCC is free by default
# to contract a*b+c into a fused multiply-add, which rounds once instead of
# twice: a better answer, and a different one depending on what the compiler
# chose to fuse. main/wfest has to produce the same Remaining Energy curve on
# the Monitor as the host harness proves it produces off the bike, so the
# licence is withdrawn here as well.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties(
        "wfest/wfest.c" "wfdecode/wfdecode.c" "${WF_GEN_C}"
        PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
endif()
