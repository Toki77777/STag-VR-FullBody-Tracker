if (VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/ManfredStoiber/stag.git
    REF 6214edc03095b6c488d3d95d866aa1a84228e574
)

# upstream CMakeLists.txt builds an example executable and has no install
# rules, replace it with a library-only build that exports a cmake config
file(COPY_FILE
    "${CMAKE_CURRENT_LIST_DIR}/stag-lib-CMakeLists.txt"
    "${SOURCE_PATH}/CMakeLists.txt")

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/stag")
vcpkg_copy_pdbs()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
