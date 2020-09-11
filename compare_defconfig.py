#!/usr/bin/env python

import sys

if len(sys.argv) < 3:
  raise Exception("Supply two defconfig files as arguments")

def parse(path):
  result = {}
  with open(path, 'r') as f:
    for line in f:
      line = line.strip()
      if line.startswith("CONFIG_"):
        try:
          name, value = line.split("=")
          result[name] = value
        except:
          print(f"FAILED TO SPLIT LINE: {line}")
  return result


options_1 = parse(sys.argv[1])
options_2 = parse(sys.argv[2])

common_options = list(filter(lambda x: x in options_2.keys(), options_1.keys()))
different_common = list(filter(lambda x: options_1[x] != options_2[x], common_options))
unique_1 = list(filter(lambda x: not x in options_2.keys(), options_1.keys()))
unique_2 = list(filter(lambda x: not x in options_1.keys(), options_2.keys()))

different_common.sort()
unique_1.sort()
unique_2.sort()

print("Different options:")
for opt in different_common:
  print(f"\t{opt}: \'{options_1[opt]}\' vs \'{options_2[opt]}\'")

print("\n\nUnique in 1:")
for opt in unique_1:
  print(f"\t{opt}: \'{options_1[opt]}\'")

print("\n\nUnique in 2:")
for opt in unique_2:
  print(f"\t{opt}: \'{options_2[opt]}\'")
