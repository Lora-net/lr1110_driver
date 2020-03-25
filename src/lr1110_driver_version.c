#include "lr1110_driver_version.h"

#define STR_HELPER( x ) #x
#define STR( x ) STR_HELPER( x )

#define LR1110_DRIVER_VERSION_FULL \
    "v" STR( LR1110_DRIVER_VERSION_MAJOR ) "." STR( LR1110_DRIVER_VERSION_MINOR ) "." STR( LR1110_DRIVER_VERSION_PATCH )

const char* lr1110_driver_version_get_version_string( void )
{
    return ( const char* ) LR1110_DRIVER_VERSION_FULL;
}
