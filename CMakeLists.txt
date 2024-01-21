cmake_minimum_required (VERSION 2.8)
set(CMAKE_CXX_STANDARD 17)
set(SOURCE 
src/ParserDecPOMDPSparse.cpp
src/ParserPOMDPSparse.cpp
src/SimModel.cpp
src/ExtendedGenerativeModel.cpp
src/BeliefParticles.cpp
src/Planner.cpp
src/MCJESP.cpp
src/FSC.cpp
src/TreeNode.cpp
src/Utils.cpp
# src/ExplicitEvaluation/AlphaVector.cpp
# src/ExplicitEvaluation/BestResponseMomdpModelSparse.cpp
# src/ExplicitEvaluation/ExplicitEvaluationUtils.cpp

)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# CHOOSE between:
# 1- flags for debugging
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g")
# and 
# 2- flags for fast execution
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# if ( CMAKE_COMPILER_IS_GNUCC ) #
if ( MSVC )
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
else()
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
endif()


project (MCJESP)
# add_executable(MCJESP main.cpp ${SOURCE})
add_executable(RunExperiments RunExperiments.cpp ${SOURCE})

