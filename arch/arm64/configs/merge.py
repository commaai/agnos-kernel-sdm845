#!/usr/bin/env python
a1 = set(open("tici_defconfig").read().split("\n"))
a2 = set(open("tici_defconfig_old").read().split("\n"))

for x in a1:
  if x not in a2 and "=y" in x:
    print(x)


