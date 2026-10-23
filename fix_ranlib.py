# Windows Device Guard / Smart App Control blocks arm-none-eabi-gcc-ranlib.exe
# on this machine. Plain arm-none-eabi-ranlib does the same job (no LTO here).
# Library builders (e.g. Wire) clone the env early, so patch those too.
Import("env")
env.Replace(RANLIB="arm-none-eabi-ranlib")
for lb in env.GetLibBuilders():
    lb.env.Replace(RANLIB="arm-none-eabi-ranlib")
