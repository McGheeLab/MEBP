#!/usr/bin/python3

from modules.m2 import muwp

# muwp>> load all
muwp.load_config()
muwp.load_plate()

# To do both:           muwp>> create all
muwp.create_locups()  # muwp>> create lps
muwp.create_brecs()   # muwp>> create brecs

# Start the hui.
muwp.hui_main()


