#!/usr/bin/env python

import sys

from biomech_gui.gui import ExoControlWidget
from rqt_gui.main import Main

plugin = 'biomech_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
