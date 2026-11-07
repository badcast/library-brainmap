find_package(OpenGL QUIET)
find_package(glfw3 QUIET)
if( NOT TARGET glfw AND NOT glfw3_FOUND )
    find_package(PkgConfig QUIET)
    if( PkgConfig_FOUND )
        pkg_check_modules(GLFW QUIET IMPORTED_TARGET glfw3)
    endif()
endif()

if( OpenGL_FOUND AND (TARGET glfw OR glfw3_FOUND OR TARGET PkgConfig::GLFW) )
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

    if(TARGET glfw)
        target_link_libraries(imgui PUBLIC OpenGL::GL glfw)
    elseif(glfw3_FOUND)
        target_link_libraries(imgui PUBLIC OpenGL::GL glfw)
    else()
        target_link_libraries(imgui PUBLIC OpenGL::GL PkgConfig::GLFW)
    endif()
else()
    message(STATUS "OpenGL or GLFW3 not found: skipping imgui and guitest targets")
endif()
