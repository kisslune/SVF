set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(DEFINED IN_SOURCE_BUILD)
  add_llvm_tool(graphgen graphGen.cpp)
else()
  add_executable(graphgen graphGen.cpp)

  target_link_libraries(graphgen SvfLLVM ${llvm_libs} Threads::Threads)

  set_target_properties(graphgen PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                       ${CMAKE_BINARY_DIR}/bin)
endif()
