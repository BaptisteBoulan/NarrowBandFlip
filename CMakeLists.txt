# Setup CMAKE for the project
cmake_minimum_required(VERSION 3.10)
project(renderze)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include GLFW, GLAD and GLM
include(FetchContent)

# GLFW
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG latest
)
FetchContent_MakeAvailable(glfw)

# GLAD
FetchContent_Declare(
  glad
  GIT_REPOSITORY https://github.com/Dav1dde/glad.git
  GIT_TAG master
)
FetchContent_MakeAvailable(glad)

# GLM
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG        0.9.9.8
)
FetchContent_MakeAvailable(glm)

# Add source and header files to the executable
add_executable(renderze 

    src/main.cpp
    src/shader.cpp
    src/simulation.h
    src/simulation.cpp
    src/camera.cpp
    
    src/frameRecording/stb_image_write.h
    src/frameRecording/frameRecorder.h
    src/frameRecording/frameRecorder.cpp
    
    )

target_link_libraries(renderze PRIVATE glfw glad)

if(WIN32)
  target_link_libraries(renderze PRIVATE opengl32)
endif()

target_include_directories(renderze PRIVATE ${glm_SOURCE_DIR})
