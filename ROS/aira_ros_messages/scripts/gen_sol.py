#!/usr/bin/env python

## ROS message source code generation for Solidity
##
## Converts ROS .msg files in a package into Solidity source code implementations.

import genmsg.template_tools
import sys

msg_template_map = { 'msg.sol.template':'@NAME@.sol' }

if __name__ == "__main__":
    genmsg.template_tools.generate_from_command_line_options(sys.argv,
                                                             msg_template_map, None)
