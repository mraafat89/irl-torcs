import os
import sys
print os.path.dirname(os.path.realpath(sys.argv[0]))
print os.path.realpath(__file__)
print os.path.dirname(sys.argv[0])

