add_executable(profile main.cpp)
target_link_libraries(profile PRIVATE project_warnings project_options
        catch_main loader)
target_include_directories(profile PRIVATE ../src ../extern/tinygltf)
