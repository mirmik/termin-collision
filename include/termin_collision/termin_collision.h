#pragma once

#ifdef _WIN32
    #ifdef TERMIN_COLLISION_EXPORTS
        #define TERMIN_COLLISION_API __declspec(dllexport)
    #else
        #define TERMIN_COLLISION_API __declspec(dllimport)
    #endif
#else
    #define TERMIN_COLLISION_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

TERMIN_COLLISION_API const char* termin_collision_version(void);

#ifdef __cplusplus
}
#endif
