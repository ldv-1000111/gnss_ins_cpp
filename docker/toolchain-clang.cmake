set(CMAKE_C_COMPILER   clang)
set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_EXE_LINKER_FLAGS_INIT    "-fuse-ld=lld")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-fuse-ld=lld")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-fuse-ld=lld")

add_compile_options(-stdlib=libstdc++)
add_link_options(-stdlib=libstdc++)

set(CMAKE_CXX_FLAGS_INIT "-Wall -Wextra -Wpedantic -Wno-unused-parameter")
