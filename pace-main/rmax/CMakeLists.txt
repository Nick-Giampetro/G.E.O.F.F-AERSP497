cmake_minimum_required(VERSION 3.5)

# note that it looks like one cannot have empty "source_folder" directories

dbp_dependancy(gcs.db rmax/serial.dbh rmax/datalink.dbh rmax/gps.dbh rmax/joyinputs.db)
dbp_dependancy(si.db rmax/serial.dbh rmax/datalink.dbh rmax/gps.dbh rmax/wmm.dbh)
dbp_dependancy(controller.db )
dbp_dependancy(history.db rmax/serial.dbh)
dbp_dependancy(onboard.db rmax/serial.dbh rmax/datalink.dbh)
dbp_dependancy(navigation.db rmax/wmm.dbh)
dbp_dependancy(scene.db )
dbp_dependancy(sensors.db rmax/serial.dbh rmax/rangeFinder.dbh rmax/gps.dbh )
dbp_dependancy(si.db rmax/serial.dbh rmax/gps.dbh rmax/datalink.dbh rmax/rangeFinder.dbh  )

addsrc( DEFAULT           "main"                                 TARGETS rmax SOURCES main.cpp main.h root.db rmax.dbh )
addsrc( DEFAULT           "gcs(offboard)"                        TARGETS rmax SOURCES joyinputs.db joyinputs.h joyinputs.c flightplan.dbh)
addsrc( DEFAULT           "gcs(offboard)"                        TARGETS rmax SOURCES gcs.db gcs.h gcs.c )
addsrc( DEFAULT           "gcs(offboard)"                        TARGETS rmax SOURCES panel.h panel.c )
addsrc( DEFAULT           "graphics(3D)"                         TARGETS rmax SOURCES scene.db scene.h scene.c )
addsrc( DEFAULT           "graphics(3D)/objects"                 TARGETS rmax SOURCES scene_multirotor.h scene_multirotor.c generic.db )
addsrc( DEFAULT           "graphics(3D)"                         TARGETS rmax SOURCES texture.h texture.c )
addsrc( DEFAULT           "onboard"                              TARGETS rmax SOURCES commands.h commands.cpp )
addsrc( DEFAULT           "onboard"                              TARGETS rmax SOURCES onboard.db onboard.h onboard.cpp )
addsrc( DEFAULT           "onboard/comm"                         TARGETS rmax SOURCES datalink.dbh datalink.h datalink.cpp )
addsrc( DEFAULT           "onboard/sensors"                      TARGETS rmax SOURCES sensors.db sensors.h sensors.cpp )
addsrc( DEFAULT           "onboard/sensors/gps"                  TARGETS rmax SOURCES gps.dbh novatel.h novatel.cpp )
addsrc( DEFAULT           "onboard/sensors/agl"                  TARGETS rmax SOURCES aglSensor.h aglSensor.cpp)
addsrc( DEFAULT           "onboard/control"                      TARGETS rmax SOURCES controller.db controller.h controller.cpp )
addsrc( DEFAULT           "onboard/control"                      TARGETS rmax SOURCES realScene.db )
addsrc( DEFAULT           "onboard/nav"                          TARGETS rmax SOURCES navigation.db navigation.h navigation.cpp )
addsrc( DEFAULT           "onboard/sensors/agl"                  TARGETS rmax SOURCES dted.db dted.h dted.cpp )
addsrc( DEFAULT           "sim interface"                        TARGETS rmax SOURCES collisiondetect.h collisiondetect.cpp )
addsrc( DEFAULT           "sim interface"                        TARGETS rmax SOURCES si.db si.h si.c )
addsrc( DEFAULT           "utils/network"                        TARGETS rmax SOURCES checksum.h checksum.c )
addsrc( DEFAULT           "utils/network"                        TARGETS rmax SOURCES serial.dbh serial.h serial.cpp )
addsrc( DEFAULT           "utils/logging"                        TARGETS rmax SOURCES logger.db logger.h logger.cpp )
addsrc( DEFAULT           "utils/world database"                 TARGETS rmax SOURCES wdb.dbh wdb.db wdb.h wdb.cpp wmm.dbh )
addsrc( DEFAULT           "utils/math"                           TARGETS rmax SOURCES matrix.h matrix.cpp )
addsrc( DEFAULT           "utils/math"                           TARGETS rmax SOURCES vector.h vector.c )
addsrc( DEFAULT           "vehicle models"                       TARGETS rmax SOURCES gear.db gear.h gear.c motion.db motion.h motion.c xforms.h xforms.c )
addsrc( DEFAULT           "vehicle models/multirotor"            TARGETS rmax SOURCES motion_multirotor.h motion_multirotor.c aeroMultirotor.h aeroMultirotor.c rotorsMultirotor.c rotorsMultirotor.h )
addsrc( DEFAULT           "utils/network"                        TARGETS rmax SOURCES ether.h ether.cpp )

# put all the generated db code in a folder
source_group("dbcode" FILES ${rmax_ref} ${rmax_def} ${rmax_dbc}  )

# workversion that generates the workversion.h file
set(WORKVERSIONEXE ${CMAKE_CURRENT_SOURCE_DIR}/workversion.bat)
ADD_CUSTOM_TARGET(workversion ALL "${WORKVERSIONEXE}")

set(rmaxtargets)

#rmax
add_executable(rmax ${rmax_src} ${rmax_ref} ${rmax_def} ${rmax_dbc})
setuptarget(rmax)
adddefine(rmax "GTMAXSIM")

target_link_libraries(rmax ${ESIM_LIBRARY} ${rmax_LIBS} ${EXTRA_LIBS})

add_dependencies(rmax workversion)
list(APPEND rmaxtargets rmax)
install(TARGETS rmax RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib )

message("rmax targets = ${rmaxtargets}")
message("current source dir = ${CMAKE_CURRENT_SOURCE_DIR}")
# put the executables we generate i.e, rmax in the work/rmax directory
set_target_properties(${rmaxtargets} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(${rmaxtargets} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})