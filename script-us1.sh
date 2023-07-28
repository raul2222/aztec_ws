#!/bin/bash

gnome-terminal -- bash -c "cd /home/remote/aztec_voice && source env/bin/activate && python service.py; exec bash"

gnome-terminal -- bash -c "cd /home/remote/ears_Aztec && source env/bin/activate && python main.py; exec bash"