### TODO
# fix the iplib issue
# fix this _USE_MATH_DEFINES
# dbp still generates crap indices when compiled in debug mode
# figure out good way to include ether.h,cpp etc
# do the onboard and onboard2 flags gtmaxsim flags
# I blindly add the _M_IX86 definition because PvAPI.h needs either this one or one other to compile correctly
#     This is supposed to be automatically set in Visual Stuio
# For Halcon I copied over the include/cpp/*.h into the include directory for halcon I would like to leave it default
# PvRegIO.h is modified to iclude prosilicage/PvAPI.h this should be changed to remove the prosilicage prefix
# right now sensoray314 driver includes and libs and dlls exist in 3rdparty/sensoray314, however they also exist in gcv/sensoray314 which needs to be removed
#    and the include paths in the sensoray314.cpp and sensoray314linux.cpp corrected.
# in 64bit environemnts (HAVE_64_BIT==1), setting BUILD_NATIVE_64_BIT to OFF doesn't seem to have an effect on DBPEXE,
#    the path still will be ../x86_linux64/bin and not ../x86_linux/ as given throught the (correct) BINDIR

cmake_minimum_required(VERSION 3.8)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT rmax)
#set_property(GLOBAL PROPERTY VS_STARTUP_PROJECT rmax)
if(WIN32)
  set(CMAKE_GENERATOR_PLATFORM win32)
endif()
project(gtmax)
set(GTUAVLAB ${CMAKE_SOURCE_DIR})
message("Sources are used from ${GTUAVLAB}. ${VS_STARTUP_PROJECT}")

#### dbp_dependancy ###########################################################
#     file - name of dbfile like rotors.db
#     ARGN - names of dbh files this db file depends on
# Example : dbp_dependancy(gcv.db gcv.dbh rmax/serial.dbh)
# This macro first searches the current path i.e., same directory as the db file
# for the dbh file, if it does not find it, it searches, in order the
# paths given in DBPINCLUDEPATHS
# So for example if DBPINCLUDEPATHS is set to "e:/work;e:/work/central/x86-win32/include"
# and gcv.db is in e:/work/gcv
# For gcv.dbh the following will be searcheed
#     e:/work/gcv/gcv.dbh
#     e:/work/gcv.dbh
#     e:/work/central/x86-win32/include/gcv.dbh
# For rmax/serial.dbh the following will be searched
#     e:/work/gcv/rmax/serial.dbh
#     e:/work/rmax/serial.dbh
#     e:/work/central/x86-win32/include/rmax/serial.dbh
MACRO(dbp_dependancy file)
#message("finding dep for " ${file})
    foreach(depfile ${ARGN})
#message(" looking for " ${depfile})
        #if(EXISTS ${depfile})
        #    set(${file}_deps ${${file}_deps} ${depfile})
#message("   *found it at home, " ${depfile})
        #else()
            foreach(path ${DBPINCLUDEPATHS})
#message("  trying " ${path}/${depfile}) 
                if(EXISTS ${path}/${depfile})
                    set(${file}_deps ${${file}_deps} ${path}/${depfile})
#message("   *found it at other " ${path}/${depfile})
                    break()
                endif()
            endforeach()
        #endif()
#message("   ->set to " ${${file}_deps})
    endforeach()

ENDMACRO(dbp_dependancy)

#### parse_arguments ##########################################################
# allows you to parse arguments in the same way cmake does
MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
  SET(DEFAULT_ARGS)
  FOREACH(arg_name ${arg_names})
    SET(${prefix}_${arg_name})
  ENDFOREACH(arg_name)
  FOREACH(option ${option_names})
    SET(${prefix}_${option} FALSE)
  ENDFOREACH(option)

  SET(current_arg_name DEFAULT_ARGS)
  SET(current_arg_list)
  FOREACH(arg ${ARGN})
    SET(larg_names ${arg_names})
    LIST(FIND larg_names "${arg}" is_arg_name)
    IF (is_arg_name GREATER -1)
      SET(${prefix}_${current_arg_name} ${current_arg_list})
      SET(current_arg_name ${arg})
      SET(current_arg_list)
    ELSE (is_arg_name GREATER -1)
      SET(loption_names ${option_names})
      LIST(FIND loption_names "${arg}" is_option)
      IF (is_option GREATER -1)
             SET(${prefix}_${arg} TRUE)
      ELSE (is_option GREATER -1)
             SET(current_arg_list ${current_arg_list} ${arg})
      ENDIF (is_option GREATER -1)
    ENDIF (is_arg_name GREATER -1)
  ENDFOREACH(arg)
  SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)

#### adddefine ################################################################
#     target - the name of the target to which you want to add this define
#     defines - the list of defines to add to specified target
MACRO(adddefine target defines)
    get_target_property( _propval ${target} COMPILE_DEFINITIONS)
    set(_propval "${_propval};${defines}")
    set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${_propval}")
ENDMACRO()

#### addcomp ##################################################################
#    Usage : addcomp(name ON|OFF description TARGETS target1 target2 DEFINES define1 define2)
#    name   - name of feature/component
#    ON|OFF - is this component on or off by default
#    description - a simple description of the component
#    TARGETS     - a list of targets to which to add these components to
#    DEFINES     - in Addition to -DNAME and -DHAVE_NAME what defines
#                   need to be added when this component is included in a target
#    Outputs:
#    ${target}_COMPONENTS - appends component to this variable
#    ${name}_ONOFF        - indicates whether this component is on or off by default
#    WITH_${name}         - if this component is supposed to be on
#    ${name}_DEFINES      - list additional #defines if component is on
#
#    .This macro adds the component with name "name" to the list of components
#    to each target listed after TARGETS. By adding a component to a target we mean
#    it simply adds it to the ${target}_COMPONENTS list
#    if the component is specified to be on by default
MACRO(addcomp)
    parse_arguments(addcomp "TARGETS;DEFINES" "" ${ARGN})
    list(GET addcomp_DEFAULT_ARGS 0 addcomp_NAME)
    list(GET addcomp_DEFAULT_ARGS 1 addcomp_ONOFF)
    list(GET addcomp_DEFAULT_ARGS 2 addcomp_DESC)
    set(WITH_${addcomp_NAME} ${addcomp_ONOFF} CACHE BOOL "${addcomp_DESC}"  )
    if(WITH_${addcomp_NAME})
        foreach(target ${addcomp_TARGETS})
            list(APPEND ${target}_COMPONENTS ${addcomp_NAME})
            list(APPEND ${addcomp_NAME}_DEFINES    ${addcomp_DEFINES})
        endforeach()
    endif()
ENDMACRO()

MACRO(adddbh)
    foreach(dbhfile ${ARGN})
        if(${dbhfile}_deps)
            SET_SOURCE_FILES_PROPERTIES(${dbhfile} PROPERTIES OBJECT_DEPENDS ${${dbhfile}_deps} HEADER_FILE_ONLY TRUE)
            #message("${dbhfile} depends on ${${dbhfile}_deps}")
        endif()
    endforeach()
ENDMACRO()

#### adddbp ###################################################################
#    Usage : adddbp(target dbfiles)
#    target   - name of target
#    dbfiles  - the list of dbfiles to add to this target
#               need to be added when this component is included in a target
#    Outputs:
#    ${target}_db        - appends the given db files to given targets list of db files
#    ${target}_ref       - appends the given ref files to given targets list of ref files
#    ${target}_def       - appends the given def files to given targets list of def files
#    ${target}_dbc       - appends the given dbc files to given targets list of dbc files
#
#    This is the macro that specifies the custom build command for a db file.
#    command line arguments for dbp are formulated using the DBPINCLUDEPATHS variable
#    which are the paths used to look for dbh files, in order

MACRO(adddbp name)
   set(adddbp_paths) # clear this var out so no leftovers from previous calls to adddbp
   #make the command line options for dbp by prefixing each item in DBPINCLUDEPATHS with a -I
   foreach(path ${DBPINCLUDEPATHS})
       set(adddbp_paths ${adddbp_paths} "-I${path}")
   endforeach()

   foreach(dbfile ${ARGN})
      get_filename_component(fullpath ${dbfile} ABSOLUTE)
      get_filename_component(path     ${fullpath} PATH)
      get_filename_component(basename ${dbfile} NAME_WE)
      set(reffile ${path}/${basename}_ref.h)
      set(deffile ${path}/${basename}_def.c)
      set(dbcfile ${path}/${basename}_db.c)

      if(${dbfile}_deps)
          #message("${dbfile} - depends on ${${dbfile}_deps}")
      endif()

      ADD_CUSTOM_COMMAND(
         OUTPUT ${reffile} ${deffile} ${dbcfile}
         COMMAND ${DBPEXE} ${adddbp_paths} ${path}/${basename}.db
         MAIN_DEPENDENCY ${path}/${basename}.db
         WORKING_DIRECTORY ${path}
         DEPENDS ${DBPEXE} ${path}/${basename}.db ${${dbfile}_deps}
      )
      set(${name}_db  ${${name}_db}  ${dbfile} )
      set(${name}_ref ${${name}_ref} ${reffile} )
      set(${name}_def ${${name}_def} ${deffile} )
      set(${name}_dbc ${${name}_dbc} ${dbcfile} )
   endforeach(dbfile)
ENDMACRO(adddbp)

#### setuptarget ##############################################################
#    Usage : setuparget(target)
#    target   - name of target
#    Outputs:
#        ${target}_LIBS - list of libraries that this target will have to link to
#
#    This macro examines the list of components in ${target}_COMPONENTS
#    and any include paths, defines required by the target
#    This includes and defines needed are set on the target directly
#    However the libraries that have to be linked in is set to ${target}_LIBS

macro(setuptarget target)
    list(REMOVE_DUPLICATES ${target}_COMPONENTS)

    # must empty out these lists
    set(_includes)
    set(_libs)
    set(_defines)


    foreach(comp ${${target}_COMPONENTS})
        #message("${comp} ${${comp}_FOUND} ${WITH_${comp}} includes ${${comp}_INCLUDE_DIRS} ${${comp}_INCLUDE_DIR} defines ${${comp}_DEFINES} libs ${${comp}_DEFINES} ${${comp}_LIBRARIES} ${${comp}_LIBRARY}")
        if(WITH_${comp} )
            if(${comp}_INCLUDE_DIRS)
                list(APPEND _includes ${${comp}_INCLUDE_DIRS})
            elseif(${comp}_INCLUDE_DIR)
                list(APPEND _includes ${${comp}_INCLUDE_DIR})
            endif()

            if(${comp}_LIBRARIES)
                list(APPEND _libs ${${comp}_LIBRARIES})
            elseif(${comp}_LIBRARY)
                list(APPEND _libs ${${comp}_LIBRARY})
            endif()

            list(APPEND _defines "HAVE_${comp}")
            if(${comp}_DEFINES)
                list(APPEND _defines ${${comp}_DEFINES})
            endif()
        endif()
    endforeach()

    set(${target}_LIBS ${_libs})

    #message("${target}_COMPONENTS = ${${target}_COMPONENTS}")
    #message("${target}_INCLUDES = ${_includes}")
    #message("${target}_DEFINES  = ${_defines}")
    #message("${target}_LIBS     = ${_libs}")

    set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "${_defines}")
    #set_target_properties(${target} PROPERTIES LINK_FLAGS "${_libs}")
    include_directories(${_includes})
endmacro()

#### addsrc ###################################################################
#    Usage : addsrc(module sourcegroup TARGETS target1 target2.. SOURCES src1 src2)
#    module   - if WITH_${module} is set, the sources are added to each target listed
#    sourcegroup - the folder in which the source file will appear in an IDE
#    TARGETS     - list of targets to which this source code will be added
#    SOURCES     - list of source files including .h, .cpp, .db and .dbh files
#    Output:
#        ${target}_ref, ${target_def}, ${target_src}, ${target_db}
#
#    Perhaps the most frequently used macro. It should be read as if WITH_${module} is ON
#    then we will go ahead and append the source files src1, src2 etc.. to each of the
#    given targets. If a db file is listed then the appropriate ref, def, db etc files
#    will be added to the targets and a custom build command for each of the db files
#    will also be automatically generated

macro(addsrc)
    parse_arguments(addsrc "TARGETS;SOURCES" "" ${ARGN})
    list(GET addsrc_DEFAULT_ARGS 0 addsrc_MODULE)
    list(GET addsrc_DEFAULT_ARGS 1 addsrc_SOURCEGROUP)

    #add to source group
    string(REGEX REPLACE "/" "\\\\" addsrc_SOURCEGROUP ${addsrc_SOURCEGROUP} )
    source_group("${addsrc_SOURCEGROUP}" FILES ${addsrc_SOURCES})

    # look for db files and them to all DBTARGETS
    foreach(file ${addsrc_SOURCES})
        if(${file} MATCHES ".db$")
            if(WITH_${addsrc_MODULE})
                foreach(t ${DBTARGETS})
                    adddbp(${t} "${file}")
                endforeach()
            endif()
        endif()
        if(${file} MATCHES ".dbh$")
            foreach(t ${DBTARGETS})
                adddbh(${t} "${file}")
            endforeach()
        endif()
    endforeach()
    #similarly any .db files are added usign adddbp
    # resuling in target_db target_ref target_def target_dbc etc..
    foreach(target ${addsrc_TARGETS})
        foreach(file ${addsrc_SOURCES})
            if(WITH_${addsrc_MODULE})
                set(${target}_src ${${target}_src} ${file})
            endif()
        endforeach()
    endforeach()
endmacro()

if( CMAKE_SIZEOF_VOID_P EQUAL 4 )
  # this is a 32bit machine
  message(STATUS "CMake detected the compiler is NOT for 64bit machine.")
endif()    

if(WIN32)
  if(MSVC60 OR MSVC70 OR MSVC71 OR MSVC80 OR MSVC90 )
    set(CMAKE_INSTALL_PREFIX ${GTUAVLAB}/central/x86-win32 CACHE PATH "" FORCE)
    set(BINDIR ${GTUAVLAB}/central/x86-win32)
  set(COMPILEBINDIR ${GTUAVLAB}/central/x86-win32)
  else()
    set(CMAKE_INSTALL_PREFIX ${GTUAVLAB}/central/x86-win32-vs2010 CACHE PATH "" FORCE)
    set(BINDIR ${GTUAVLAB}/central/x86-win32-vs2010)
  set(COMPILEBINDIR ${GTUAVLAB}/central/x86-win32-vs2010)
  endif()
  set(BINDIROLD ${GTUAVLAB}/central/i386-win32)
  add_definitions(-DSUL_OS_WIN32)
  add_definitions(-DPTR2INTEGER=long)
  add_definitions("-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES -DWIN32") # this should really only go with if(MSVC)
  set(EXTRA_LIBS ${EXTRA_LIBS} winmm.lib)
  message("binaries directory is set to ${BINDIR}")
endif()

add_definitions("-DHAVE_ESIM")

set(CMAKE_INCLUDE_PATH ${BINDIR}/include ${BINDIR}/3rdparty/opengl/include ${BINDIR}/3rdparty/pthreads/include )
set(CMAKE_LIBRARY_PATH ${BINDIR}/lib     ${BINDIR}/3rdparty/opengl/lib     ${BINDIR}/3rdparty/pthreads/lib     )

set(CMAKE_PROGRAM_PATH ${COMPILEBINDIR}/bin)
if(CMAKE_CROSS_COMPILE_TARGET_INCLUDE)
	include_directories("${CMAKE_BINARY_DIR}" "${GTUAVLAB}" "${BINDIR}/include" "${CMAKE_CROSS_COMPILE_TARGET_INCLUDE}")
else()
	include_directories("${CMAKE_BINARY_DIR}" "${GTUAVLAB}" "${BINDIR}/include")
endif()

include(CheckFunctionExists)
#### Some functions ####
#check_function_exists(round HAVE_ROUND_FUNCTION)
#check_function_exists(sin HAVE_FINITE)
if(WIN32 AND MSVC_VERSION GREATER 1899)
  MESSAGE(STATUS "Using MSVS version 14 or greater.")
  set(HAVE_ROUND_FUNCTION ON CACHE BOOL "round function already defined?")
  set(HAVE_STRUCT_TIMESPEC ON CACHE BOOL "pthread struct timespec already defined?")
elseif(WIN32)
  set(HAVE_ROUND_FUNCTION OFF CACHE BOOL "round function already defined?")
elseif(UNIX)
  set(HAVE_ROUND_FUNCTION ON  CACHE BOOL "round function already defined?")
endif()
if(NOT HAVE_ROUND_FUNCTION)
  add_definitions(-DCUSTOM_ROUND_FUNCTION)
endif()
if(HAVE_STRUCT_TIMESPEC)
  add_definitions(-DHAVE_STRUCT_TIMESPEC)
endif()

############ Components ###########
addcomp(DEFAULT          ON   " "              TARGETS rmax )
addcomp(PTHREADS         ON   "Have PThreads"  TARGETS rmax )
addcomp(OPENGL           ON   "Have OpenGL?"   TARGETS rmax )
addcomp(GLUT             ON   " "              TARGETS rmax )
addcomp(SOCKETS          ON   " "              TARGETS rmax )

# With or without graphics ?
if(WITH_OPENGL)
  find_package(OpenGL REQUIRED)
endif()

if(WITH_GLUT)
  find_package(GLUT REQUIRED)
endif()

# sockets
if(WIN32)
  # set(SOCKETS2_LIBRARY ws2_32 CACHE STRING "sockets2 library")
  # mark_as_advanced(SOCKETS2_LIBRARY)
  set(SOCKETS_LIBRARY wsock32 CACHE STRING "sockets library")
  mark_as_advanced(SOCKETS_LIBRARY)
  set(IPHLPAPI_LIBRARY iphlpapi)
  mark_as_advanced(IPHLPAPI_LIBRARY)
endif()
if(SOCKETS_LIBRARY)
  set(SOCKETS_FOUND ON)
endif()

# Always with PThreads
if(WIN32)
  find_path(PTHREADS_INCLUDE_DIR NAMES pthread.h)
  find_library(PTHREADS_LIBRARY NAMES pthreadVCE2)
else()
  find_path(PTHREADS_INCLUDE_DIR NAMES pthread.h)
  find_library(PTHREADS_LIBRARY NAMES pthread)
endif()
mark_as_advanced(PTHREADS_INCLUDE_DIR PTHREADS_LIBRARY)
if(PTHREADS_INCLUDE_DIR AND PTHREADS_LIBRARY)
  set(PTHREADS_FOUND ON)
  set(PTHREADS_INCLUDE_DIRS ${PTHREADS_INCLUDE_DIR})
  set(PTHREADS_LIBRARIES ${PTHREADS_LIBRARY})
endif()

##### List all targets that might use db files here
# this is needed because db files are compiled into all targets to prserve the variable tree
# so what this does is if a db files appears in an addsrc command whether or not that particulr component is enabled or not
# the dbfile will be compiled in
set(DBTARGETS rmax)
# these are the paths that dbp will look for dbh files, order is important
set(DBPINCLUDEPATHS ${DBPINCLUDEPATHS} "${GTUAVLAB}" "${BINDIR}/include")

find_program(DBPEXE NAMES dbp PATHS ${COMPILEBINDIR}/bin NO_DEFAULT_PATH)
find_library(ESIM_LIBRARY esim)
add_subdirectory(rmax)