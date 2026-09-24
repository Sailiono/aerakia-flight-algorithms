/**
 * @file version.h
 * @brief Compile-time Aerakia public API version.
 */

#ifndef AERAKIA_VERSION_H
#define AERAKIA_VERSION_H

#define AERAKIA_VERSION_MAJOR 0
#define AERAKIA_VERSION_MINOR 3
#define AERAKIA_VERSION_PATCH 0
#define AERAKIA_VERSION_STRING "0.3.0"

/** Pack a semantic version into a monotonically comparable integer. */
#define AERAKIA_MAKE_VERSION(major, minor, patch) \
    ((((major) & 0xff) << 16) | (((minor) & 0xff) << 8) | ((patch) & 0xff))

#define AERAKIA_VERSION_CODE \
    AERAKIA_MAKE_VERSION(AERAKIA_VERSION_MAJOR, AERAKIA_VERSION_MINOR, AERAKIA_VERSION_PATCH)

#define AERAKIA_VERSION_AT_LEAST(major, minor, patch) \
    (AERAKIA_VERSION_CODE >= AERAKIA_MAKE_VERSION((major), (minor), (patch)))

#endif
