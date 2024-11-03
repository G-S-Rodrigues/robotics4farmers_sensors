add_library(gnss_compass_utils STATIC
src/an_packet_protocol.c
src/ins_packets.c
)

target_include_directories(gnss_compass_utils
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)