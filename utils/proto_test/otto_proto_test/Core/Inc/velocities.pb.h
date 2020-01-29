/* Automatically generated nanopb header */
/* Generated by 0.4.1-dev */

#ifndef PB_VELOCITIES_PB_H_INCLUDED
#define PB_VELOCITIES_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Velocities {
    float linear_vel;
    float angular_vel;
} Velocities;


/* Initializer values for message structs */
#define Velocities_init_default                  {0, 0}
#define Velocities_init_zero                     {0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Velocities_linear_vel_tag                1
#define Velocities_angular_vel_tag               2

/* Struct field encoding specification for nanopb */
#define Velocities_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    linear_vel,        1) \
X(a, STATIC,   SINGULAR, FLOAT,    angular_vel,       2)
#define Velocities_CALLBACK NULL
#define Velocities_DEFAULT NULL

extern const pb_msgdesc_t Velocities_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Velocities_fields &Velocities_msg

/* Maximum encoded size of messages (where known) */
#define Velocities_size                          10

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
