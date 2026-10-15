# ESP-IDF component (also used by Board A firmware). Host unit tests use the Makefile instead.
idf_component_register(SRCS "src/atlas_dock_fsm.c" "src/atlas_pi.c" "src/atlas_proto.c"
                       INCLUDE_DIRS "include")
