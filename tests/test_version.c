#include <aerakia/version.h>

#include <string.h>

#if AERAKIA_VERSION_MAJOR != AERAKIA_CMAKE_VERSION_MAJOR
# error "public major version does not match CMake project version"
#endif

#if AERAKIA_VERSION_MINOR != AERAKIA_CMAKE_VERSION_MINOR
# error "public minor version does not match CMake project version"
#endif

#if AERAKIA_VERSION_PATCH != AERAKIA_CMAKE_VERSION_PATCH
# error "public patch version does not match CMake project version"
#endif

#if AERAKIA_VERSION_CODE != AERAKIA_MAKE_VERSION(0, 3, 0)
# error "packed public version is incorrect"
#endif

#if !AERAKIA_VERSION_AT_LEAST(0, 3, 0)
# error "current version must satisfy its own minimum-version check"
#endif

#if AERAKIA_VERSION_AT_LEAST(0, 4, 0)
# error "minimum-version check must reject a newer API"
#endif

int main(void)
{
    return strcmp(AERAKIA_VERSION_STRING, AERAKIA_CMAKE_VERSION_STRING) == 0 ? 0 : 1;
}
