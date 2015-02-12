#! /bin/sh

while [ 1 ]
do
    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- true
- false
- false
- false
- false
- false
- false
- false" > /dev/null) & 

    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- true
- false
- false
- false
- false
- false
- false" > /dev/null) &

    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- false
- true
- false
- false
- false
- false
- false" > /dev/null) &

    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- false
- false
- true
- false
- false
- false
- false" > /dev/null) &

    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- false
- false
- false
- true
- false
- false
- false" > /dev/null) &

    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- false
- false
- false
- false
- true
- false
- false" > /dev/null) &


    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- false
- false
- false
- false
- false
- true
- false" > /dev/null) &


    sleep 0.25

    $(rostopic pub --once  /lift/bumpers/state rose20_platform/bumpers_state "bumper_count: 8
bumper_states:
- false
- false
- false
- false
- false
- false
- false
- true" > /dev/null) &
done
