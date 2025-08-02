cmake_minimum_required(VERSION 3.22)

# Only build bindings when using Emscripten
if(EMSCRIPTEN)
    # Create the bindings library
    add_executable(box2d_bindings box2d_bindings.cpp)
    
    # Link with Box2D
    target_link_libraries(box2d_bindings box2d)
    
    # Include Box2D headers
    target_include_directories(box2d_bindings PRIVATE 
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
    )
    
    # Emscripten-specific settings - disable threading for web bindings
    set_target_properties(box2d_bindings PROPERTIES
        SUFFIX ".js"
        LINK_FLAGS "--bind -s MODULARIZE=1 -s EXPORT_NAME='Box2D' -s ENVIRONMENT=web -s ALLOW_MEMORY_GROWTH=1 -s ASSERTIONS=1 -O3 -s EXPORTED_RUNTIME_METHODS=['FS'] -s USE_PTHREADS=0"
        COMPILE_FLAGS "-s USE_PTHREADS=0"
    )
    
    # Generate TypeScript definitions
    add_custom_command(TARGET box2d_bindings POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/box2d.d.ts ${CMAKE_CURRENT_BINARY_DIR}/box2d.d.ts
        COMMENT "Copying TypeScript definitions"
    )
    
endif()