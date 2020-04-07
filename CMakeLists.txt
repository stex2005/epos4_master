file(GLOB MY_PROJECT_HEADERS *.h epos4_driver/*.h)
file(GLOB MY_PROJECT_SOURCES *.cpp *.c epos4_driver/*.cpp)

INCLUDE_DIRECTORIES(epos4_driver)

add_executable(master ${MY_PROJECT_SOURCES} ${MY_PROJECT_HEADERS})
target_link_libraries(master ethercat_driver esmacat)
install(TARGETS master DESTINATION ./bin)

