#!/bin/sh

cmd=""
for arg in "$@"
do
    cmd="${cmd} $arg"
done

bash -ic "$cmd"
