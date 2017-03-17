#-----------------------------------------------------------------------------
#
#  CMake Config
#
#  OSM Data Anomaly Detection
#
#-----------------------------------------------------------------------------

add_executable(odad-find-colocated-nodes odad-find-colocated-nodes.cpp)
target_link_libraries(odad-find-colocated-nodes ${OSMIUM_LIBRARIES} sqlite3)
install(TARGETS odad-find-colocated-nodes DESTINATION bin)

add_executable(odad-find-orphans odad-find-orphans.cpp)
target_link_libraries(odad-find-orphans ${OSMIUM_LIBRARIES} sqlite3)
install(TARGETS odad-find-orphans DESTINATION bin)

add_executable(odad-find-relation-problems odad-find-relation-problems.cpp)
target_link_libraries(odad-find-relation-problems ${OSMIUM_IO_LIBRARIES} sqlite3)
install(TARGETS odad-find-relation-problems DESTINATION bin)

add_executable(odad-find-unusual-tags odad-find-unusual-tags.cpp)
target_link_libraries(odad-find-unusual-tags ${OSMIUM_IO_LIBRARIES} sqlite3)
install(TARGETS odad-find-unusual-tags DESTINATION bin)

add_executable(odad-find-way-problems odad-find-way-problems.cpp)
target_link_libraries(odad-find-way-problems ${OSMIUM_LIBRARIES} sqlite3)
install(TARGETS odad-find-way-problems DESTINATION bin)

