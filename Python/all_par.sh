#!/bin/bash

python3 telegraph_gen.py 3 &
python3 telegraph_gen.py 2 &
python3 telegraph_gen.py 0 &
python3 telegraph_gen.py 1 &

