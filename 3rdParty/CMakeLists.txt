add_library(imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
)

find_package(OpenGL REQUIRED)
find_package(glfw3 QUIET)
if(TARGET glfw)
    target_link_libraries(imgui PUBLIC OpenGL::GL glfw)
elseif(glfw3_FOUND)
    target_link_libraries(imgui PUBLIC OpenGL::GL glfw)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)
    target_link_libraries(imgui PUBLIC OpenGL::GL PkgConfig::GLFW)
endif()
