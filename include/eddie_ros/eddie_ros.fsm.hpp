/*
 * This is an auto-generated file. Do not edit it directly.
 * 
 * FSM: eddie_ros
 * FSM Description: 
 *
 * -----------------------------------------------------
 * Usage example:
 * -----------------------------------------------------

#include "eddie_ros.fsm.hpp"

struct user_data {

};

void yyyy_behavior(struct user_data *userData, struct events *eventData) {
    // ... do something

    produce_event(eventData, E_ZZZZ);
}

void fsm_behavior(struct events *eventData, struct user_data *userData) {
    if (consume_event(eventData, E_XXXX)) {
        yyyy_behavior(userData, eventData);
    }
    ...
}

int main() {

    struct user_data userData = {};

    while (true) {
        produce_event(fsm.eventData, E_STEP);

        // run state machine, event loop
        fsm_behavior(fsm.eventData, &userData);
        fsm_step_nbx(&fsm);
        reconfig_event_buffers(&eventData);
    }

    return 0;
}

 * -----------------------------------------------------
 */

#ifndef EDDIE_ROS_FSM_HPP
#define EDDIE_ROS_FSM_HPP

#include "coord2b/functions/fsm.h"
#include "coord2b/functions/event_loop.h"

// sm states
enum e_states {
    S_START = 0,
    S_CONFIGURE,
    S_IDLE,
    S_COMPILE,
    S_EXECUTE,
    S_EXIT,
    NUM_STATES
};

// sm events
enum e_events {
    E_CONFIGURE_ENTERED = 0,
    E_CONFIGURE_EXIT,
    E_IDLE_ENTERED,
    E_IDLE_EXIT_EXECUTE,
    E_IDLE_EXIT_COMPILE,
    E_COMPILE_ENTERED,
    E_COMPILE_EXIT,
    E_EXECUTE_ENTERED,
    E_STEP,
    NUM_EVENTS
};

// sm transitions
enum e_transitions {
    T_START_CONFIGURE = 0,
    T_CONFIGURE_IDLE,
    T_IDLE_IDLE,
    T_IDLE_EXECUTE,
    T_IDLE_COMPILE,
    T_COMPILE_EXECUTE,
    T_EXECUTE_EXECUTE,
    NUM_TRANSITIONS
};

// sm reactions
enum e_reactions {
    R_E_CONFIGURE_EXIT = 0,
    R_E_IDLE_EXIT_EXECUTE,
    R_E_IDLE_EXIT_COMPILE,
    R_E_COMPILE_EXIT,
    R_E_STEP1,
    R_E_STEP2,
    R_E_STEP3,
    NUM_REACTIONS
};

// sm states
inline struct state states[NUM_STATES] = {
    {.name = "S_start"}, 
    {.name = "S_configure"}, 
    {.name = "S_idle"}, 
    {.name = "S_compile"}, 
    {.name = "S_execute"}, 
    {.name = "S_exit"} 
};

// sm transition table
inline struct transition transitions[NUM_TRANSITIONS] = {
    {
        .startStateIndex = S_START,
        .endStateIndex = S_CONFIGURE,
    }, 
    {
        .startStateIndex = S_CONFIGURE,
        .endStateIndex = S_IDLE,
    }, 
    {
        .startStateIndex = S_IDLE,
        .endStateIndex = S_IDLE,
    }, 
    {
        .startStateIndex = S_IDLE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_IDLE,
        .endStateIndex = S_COMPILE,
    }, 
    {
        .startStateIndex = S_COMPILE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_EXECUTE,
    } 
};

// sm reaction table
inline struct event_reaction reactions[NUM_REACTIONS] = {
    {
        .conditionEventIndex = E_CONFIGURE_EXIT,
        .transitionIndex = T_CONFIGURE_IDLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_IDLE_ENTERED 
        },
    }, 
    {
        .conditionEventIndex = E_IDLE_EXIT_EXECUTE,
        .transitionIndex = T_IDLE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_EXECUTE_ENTERED 
        },
    }, 
    {
        .conditionEventIndex = E_IDLE_EXIT_COMPILE,
        .transitionIndex = T_IDLE_COMPILE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_COMPILE_ENTERED 
        },
    }, 
    {
        .conditionEventIndex = E_COMPILE_EXIT,
        .transitionIndex = T_COMPILE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_EXECUTE_ENTERED 
        },
    }, 
    {
        .conditionEventIndex = E_STEP,
        .transitionIndex = T_START_CONFIGURE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_CONFIGURE_ENTERED 
        },
    }, 
    {
        .conditionEventIndex = E_STEP,
        .transitionIndex = T_IDLE_IDLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_IDLE_ENTERED 
        },
    }, 
    {
        .conditionEventIndex = E_STEP,
        .transitionIndex = T_EXECUTE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_EXECUTE_ENTERED 
        },
    } 
};

// sm event data
inline struct events eventData = {
    .numEvents = NUM_EVENTS,
    .currentEvents = new _Bool[NUM_EVENTS]{false},
    .futureEvents = new _Bool[NUM_EVENTS]{false},
};

// sm fsm struct
inline struct fsm_nbx fsm = {
    .numReactions = NUM_REACTIONS,
    .numTransitions = NUM_TRANSITIONS,
    .numStates = NUM_STATES,

    .states = states,
    .startStateIndex = S_START,
    .endStateIndex = S_EXIT,
    .currentStateIndex = S_START,

    .eventData = &eventData,
    .reactions = reactions,
    .transitions = transitions,
};

#endif // EDDIE_ROS_FSM_HPP