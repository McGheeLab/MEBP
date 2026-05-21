"""
gui.pages.hardware — Hardware Setup sub-page modules (v7.4.0-b).

Hardware Setup is now a ModePage with sub-pages partitioning a previously
monolithic configuration form. Most sub-page content lives inline in
hardware_setup.py for compactness; only the new Stage sub-page (safety
limits, ZP feedrates, axis flips, zero-cal jog moved from Settings) gets
its own module here.
"""
