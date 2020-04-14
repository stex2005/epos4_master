file(GLOB MY_PROJECT_HEADERS *.h epos4_driver/*.h)
file(GLOB MY_PROJECT_SOURCES *.cpp *.c epos4_driver/*.cpp)

include_directories(epos4_driver)

add_executable(master ${MY_PROJECT_SOURCES})
target_link_libraries(master ethercat_driver esmacat ${catkin_LIBRARIES})
install(TARGETS master DESTINATION ./bin)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


