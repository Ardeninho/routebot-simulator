#ifndef ROUTEBOT_H
#define ROUTEBOT_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define RB_MAX_STOPS 32
#define RB_STEP_MS 20u
#define RB_TRACK_M 0.40
#define RB_MAX_SPEED 0.8

typedef struct { double x, y; } Point;
typedef enum { GIVEN, NEAREST, INSERT } Method;
typedef struct { Point stops[RB_MAX_STOPS]; size_t count; } Mission;
typedef struct { size_t order[RB_MAX_STOPS + 1]; size_t count; double length; } Route;
bool plan(const Mission *mission, Method method, Route *route);

typedef struct {
    double x, y, heading;
    uint32_t sample_ms;
    bool obstacle, estop;
} Sensors;
typedef struct { double left, right; } Motors;
/* All device access crosses this boundary. Speeds are wheel m/s. */
typedef struct {
    void *context;
    bool (*read)(void *, Sensors *);
    void (*write)(void *, Motors);
} Hal;
typedef enum { IDLE, RUNNING, BLOCKED, COMPLETE, FAULT } State;
typedef enum { NO_FAULT, SENSOR_FAULT, ESTOP_FAULT, TIMEOUT_FAULT } Fault;
typedef struct {
    Mission mission;
    Route route;
    size_t target;
    State state;
    Fault fault;
    uint32_t start_ms;
    Motors command;
} Controller;
bool controller_start(Controller *, const Mission *, const Route *, uint32_t);
void controller_tick(Controller *, const Hal *, uint32_t);
const char *state_name(State);

typedef struct {
    Sensors sensors;
    Motors motors;
    uint32_t last_write_ms;
    bool dropout;
    double distance;
} Sim;
Hal sim_hal(Sim *);
void sim_step(Sim *, uint32_t now_ms);
#endif
