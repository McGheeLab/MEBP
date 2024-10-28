
if not DEFINED IS_MINIMIZED set IS_MINIMIZED=1 && start "" /min "%~pdnx0" %* && exit

python -m idlelib -r modules_u/muwp.py

exit

