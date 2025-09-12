cmake_minimum_required(VERSION 3.12)
project(TA_MAPF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(BOOST_ROOT "/home/tommaso/bin/boost_1_83_0")
set(CPLEX_ROOT_DIR "/home/tommaso/bin/ibm2212")
set(OPLDIR "${CPLEX_ROOT_DIR}/opl")
set(CPLEXDIR "${CPLEX_ROOT_DIR}/cplex")
set(CPDIR "${CPLEX_ROOT_DIR}/cpoptimizer")
set(ICUDIR "${CPLEX_ROOT_DIR}/opl")
set(CPLEXVERSION 2212)
set(CPLEXNAME cplex${CPLEXVERSION})
set(MACHINE "x86-64_linux")
set(LIBFORMAT "static_pic")
set(LDMTFLAGS "pthread")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -DIL_STD -m64 -Wall -Werror -fPIC -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -O0 -DHEURISTIC=false") #-Og  -Wno-unused-variable -DOPTIMIZE_MAKESPAN -DEXPORT_MCNF_DAG
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -DHEURISTIC=false -Wno-unused-result -Ofast -flto=4 -march=native")
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(LINKER_OPTIONS -flto=4 -Wl,--no-as-needed)
add_definitions(-DWDIR=\"/home/tommaso/TA_MAPF/\")

if (CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif ()

# check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

include_directories(SYSTEM includes/)
include_directories(SYSTEM ${BOOST_ROOT})
include_directories(SYSTEM ${OPLDIR}/include ${CPLEXDIR}/include ${CPDIR}/include)
set(SOURCE_FILES
        main.cpp
)
file(GLOB INCLUDES_SRC
        "includes/*.hpp"
        #"src/*.cpp"
        "src/AbstractTimeExpandedGraph.cpp" , "src/Agent.cpp" , "src/AStar.cpp" , "src/BoostDirectedGraph.cpp" ,
        "src/CBS.cpp" , "src/Conflict.cpp" , "src/Constraints.cpp" , "src/EdgeConstraint.cpp" ,
        "src/Environment.cpp" , "src/GridGraph.cpp" , "src/HighLevelNode.cpp" , "src/InstanceDAO.cpp" , "src/MCNF.cpp" ,
        "src/LowLevelNode.cpp" , "src/MAPFPolicy.cpp" , "src/Param.cpp" , "src/PreprocessingData.cpp" ,
        "src/QPS.cpp" , "src/State.cpp" , "src/TimeExpandedGraph.cpp" , "src/VertexConstraint.cpp"
)
link_directories(${BOOST_ROOT}/stage/lib)
link_directories(${OPLDIR}/lib/${MACHINE}/${LIBFORMAT})
link_directories(${CPLEXDIR}/lib/${MACHINE}/${LIBFORMAT})
link_directories(${CPLEXDIR}/bin/${MACHINE})
link_directories(${ICUDIR}/bin/${MACHINE})
add_executable(TA_MAPF ${SOURCE_FILES} ${INCLUDES_SRC})
target_link_libraries(TA_MAPF PRIVATE pthread opl iljs ilocplex cp concert dl ${CPLEXNAME})
