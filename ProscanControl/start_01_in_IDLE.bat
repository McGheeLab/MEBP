
if not DEFINED IS_MINIMIZED set IS_MINIMIZED=1 && start "" /min "%~pdnx0" %* && exit

:: Clicking on this opens the python IDLE
:: and leaves the cmd window also open.

:: color 3f
:: color f0

python -m idlelib

exit


