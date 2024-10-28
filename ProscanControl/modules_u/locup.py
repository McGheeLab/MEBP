#!/usr/bin/python3

from modules.m2 import locup

locup.load_config()
locup.load_pattern_data()

# There is no run() in locup, no hui.
# All is done from the python >>> prompt.
# Do this:
#   First move the stage to the north edge of the culture.
#   Then:
#   >>> locup.get_edges()
#   >>> locup.run_pattern()



