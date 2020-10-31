#!/bin/bash
sim=$1

# find the directory that the current file is in
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# run state_machine.sh in a new terminal instance
gnome-terminal -- $DIR/state_machine.sh $1