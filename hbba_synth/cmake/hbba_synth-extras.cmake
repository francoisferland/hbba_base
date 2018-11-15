cmake_minimum_required(VERSION 2.8.3)
include(CMakeParseArguments)

# add_hbba_cfg(BASE_NAME SRC [BHVR] [OLDREV])
# Automatic build of hbba_cfg with "add_hbba_cfg" macro.
# The BASENAME parameter is the target name and has to be followed by a single
# root source file for your setup.
# The BHVR argument indicates that the configuration should be generated in pure
# behavior-based mode.
# The EXTERNAL parameter specifies that base nodes should not be added to the
# generated launch file.
# The OLDREV argument activates code generation in the old revision of
# hbba_synth (unsupported).
macro(add_hbba_cfg _BASENAME _SRC)
    set(_options OLDREV BHVR)
    cmake_parse_arguments(ARG "${_options}" "" "" ${ARGN}) 
    set(_HBBA_CFG_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/hbba_cfg_out)
    file(MAKE_DIRECTORY ${_HBBA_CFG_OUTPUT_PATH})
    set(_HBBA_CFG_OUTPUT_BASENAME
        ${_HBBA_CFG_OUTPUT_PATH}/${_BASENAME}
    )
    set(_HBBA_CFG_OUTPUT
        ${_HBBA_CFG_OUTPUT_BASENAME}.launch
        ${_HBBA_CFG_OUTPUT_BASENAME}.py
    )
    unset(_src_path CACHE)
    find_file(_src_path
        NAMES ${_SRC} ${_SRC}.yaml
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}
              ${CMAKE_CURRENT_SOURCE_DIR}/hbba_cfg    # For compatibility with
                                                      # older setups.
    )
    if (NOT _src_path)
        message(SEND_ERROR "Cannot find ${_SRC} for ${_BASENAME}")
    endif()
#   find_file(HBBA_ROBOT_SRC 
#       ${ROBOT}.yaml 
#       ${PROJECT_SOURCE_DIR}/hbba_cfg 
#       ${irl1_hbba_cfg_PACKAGE_PATH}/hbba_cfg
#   )
    set(_HBBA_CFG_SRC_ALL
        ${_src_path}
    )

    list(APPEND _HBBA_CFG_OPTS "-p")
    # ARGN requires to be put in a variable before list(...) works:
    set(_vargs ${ARGN})

    list(FIND _vargs "BHVR" _opt_bhvr)
    if(NOT _opt_bhvr EQUAL -1)
        message("Building ${_BASENAME} in behavior mode.")
        list(APPEND _HBBA_CFG_OPTS "-b")
    endif()

    list(FIND _vargs "EXTERNAL" _opt_external)
    if (NOT _opt_external EQUAL -1)
        message("Building ${_BASENAME} in external mode (no base nodes).")
        list(APPEND _HBBA_CFG_OPTS "-e")
    endif()

    list(FIND _vargs "OLDREV" _opt_oldrev)
    if (NOT _opt_oldrev EQUAL -1)
        message("Building ${_BASENAME} with the old IW revision.")
        list(APPEND _HBBA_CFG_OPTS "--old-rev")
    endif()

    list(FIND _vargs "NO_EVENT_GENERATORS" _opt_no_event_generators)
    if (NOT _opt_no_event_generators EQUAL -1)
        message("Building ${_BASENAME} with no event generators.")
        list(APPEND _HBBA_CFG_OPTS "--no-event-generators")
    endif()

    list(APPEND _HBBA_CFG_OPTS "-o")

    message("Gathering HBBA dependencies for ${_BASENAME}...")
    execute_process(
        COMMAND rosrun hbba_synth hbba_synth_deps.sh ${_HBBA_CFG_SRC_ALL}
        OUTPUT_VARIABLE _HBBA_CFG_SRC_DEPS
    )

    add_custom_command(
        OUTPUT ${_HBBA_CFG_OUTPUT}
        COMMAND rosrun hbba_synth hbba_synth ${_HBBA_CFG_OPTS} ${_HBBA_CFG_OUTPUT_BASENAME} ${_HBBA_CFG_SRC_ALL}
        DEPENDS ${_HBBA_CFG_SRC_ALL} ${_HBBA_CFG_SRC_DEPS}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
    add_custom_target(${_BASENAME} ALL DEPENDS ${_HBBA_CFG_OUTPUT})

    install(FILES ${_HBBA_CFG_OUTPUT}
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/hbba_cfg_out
    )

endmacro(add_hbba_cfg)

