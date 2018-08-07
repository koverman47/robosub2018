#!/usr/bin/env python


target_depth = 2


surfacePSI = 10.44

signal = (((target_depth / 0.13197839577) + 10.44) / 12.5) + 1 / 0.0048828125

depth = ((206 * 0.0048828125 - 1) * 12.5 - surfacePSI) * 0.13197839577

print(depth)

#print(signal)
