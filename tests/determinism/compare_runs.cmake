# Runs the probe in two separate processes and fails if the hashes differ.

foreach(run first second)
  execute_process(
    COMMAND "${PROBE}"
    OUTPUT_VARIABLE hash_${run}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE result_${run})
  if(NOT result_${run} EQUAL 0)
    message(FATAL_ERROR "Probe failed: ${result_${run}}")
  endif()
endforeach()

if(hash_first STREQUAL "" OR NOT hash_first STREQUAL hash_second)
  message(FATAL_ERROR "Runs differ: '${hash_first}' vs '${hash_second}'")
endif()

message(STATUS "Both runs produced ${hash_first}")
