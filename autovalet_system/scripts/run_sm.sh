#!/bin/bash
sim=$1
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
gnome-terminal -- $DIR/state_machine.sh $1