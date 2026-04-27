project(mm-viewer)

find_package(mrpt-tclap REQUIRED)
find_package(mrpt-gui REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
    LODBuilder.cpp
    LODRenderer.cpp
  LINK_LIBRARIES
    mp2p_icp
    mrpt::tclap
    mrpt::gui
    Threads::Threads
  )

if (TBB_FOUND AND MP2PICP_USE_TBB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_HAS_TBB)
  target_link_libraries(${PROJECT_NAME} TBB::tbb)
endif()
