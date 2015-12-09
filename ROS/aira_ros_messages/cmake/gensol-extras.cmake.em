@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(GENSOL_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_sol.py")
set(GENSOL_TEMPLATE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace
set(GENSOL_BIN "${gensol_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_sol.py")
set(GENSOL_TEMPLATE_DIR "${gensol_DIR}/..")
@[end if]@

# Generate .msg->.sol for Solidity
# The generated .sol files should be added ALL_GEN_OUTPUT_FILES_sol
macro(_generate_msg_sol ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}.sol)
  set(GEN_OUTPUT_FILE ${ARG_GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENSOL_BIN} ${ARG_MSG} ${ARG_MSG_DEPS} "${GENSOL_TEMPLATE_DIR}/msg.sol.template" ${ARGN}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENSOL_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${ARG_GEN_OUTPUT_DIR}
    -e ${GENSOL_TEMPLATE_DIR}
    COMMENT "Generating Solidity code from ${ARG_PKG}/${MSG_NAME}"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_sol ${GEN_OUTPUT_FILE})

  gencpp_append_include_dirs()
endmacro()

#gensol uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_sol ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
endmacro()

macro(_generate_module_sol)
  # the macros, they do nothing
endmacro()
