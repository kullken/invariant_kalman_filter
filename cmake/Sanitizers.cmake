function(enable_sanitizers project_name)

  set(SANITIZERS "")

  option(ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" FALSE)
  if(ENABLE_SANITIZER_ADDRESS)
    list(APPEND SANITIZERS "address")
  endif()

  option(ENABLE_SANITIZER_LEAK "Enable leak sanitizer" FALSE)
  if(ENABLE_SANITIZER_LEAK)
    list(APPEND SANITIZERS "leak")
  endif()

  option(ENABLE_SANITIZER_UNDEFINED_BEHAVIOR "Enable undefined behavior sanitizer" FALSE)
  if(ENABLE_SANITIZER_UNDEFINED_BEHAVIOR)
    list(APPEND SANITIZERS "undefined")
  endif()

  option(ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" FALSE)
  if(ENABLE_SANITIZER_MEMORY AND CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    if("address" IN_LIST SANITIZERS OR "leak" IN_LIST SANITIZERS)
      message(WARNING "Memory sanitizer does not work with Address or Leak sanitizer enabled")
    else()
      list(APPEND SANITIZERS "memory")
    endif()
  endif()

  list(JOIN SANITIZERS "," LIST_OF_SANITIZERS)

  if(LIST_OF_SANITIZERS)
    if(NOT "${LIST_OF_SANITIZERS}" STREQUAL "")
      target_compile_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
      target_link_libraries(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
    endif()
  endif()

endfunction()