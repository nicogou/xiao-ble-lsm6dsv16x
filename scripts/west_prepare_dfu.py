# Copyright (c) 2019 Foundries.io
# Copyright (c) 2022 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

'''west_prepare_dfu.py'''

from west.commands import WestCommand  # your extension must subclass this
from west import log                   # use this for user output
import os
import platform
import zipfile

class WestPrepareDfu(WestCommand):

    def __init__(self):
        super().__init__(
            'prepare-dfu',               # gets stored as self.name
            'prepare the files for OTA DFU',  # self.help
            # self.description:
            '''\
This command prepares the file needed for an Over-the-Air DFU.

This command generates the .dat file necessary to flash a firmware
over-the-air with the Bluefruit Connect app.''')

    def do_add_parser(self, parser_adder):
        # This is a bit of boilerplate, which allows you full control over the
        # type of argparse handling you want. The "parser_adder" argument is
        # the return value of an argparse.ArgumentParser.add_subparsers() call.
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)
        return parser           # gets stored as self.parser

    def do_run(self, args, unknown_args):
        # This gets called when the user runs the command, e.g.:
        #
        #   $ west my-command-name -o FOO BAR

        build_path = os.path.join(os.getcwd(), 'build', 'zephyr')
        hex_path = os.path.join(os.getcwd(), 'build', 'zephyr', 'zephyr.hex')
        dest_path = os.path.join(os.getcwd(), 'build', 'zephyr', 'zephyr_ada_bl.zip')
        if not os.path.exists(hex_path):
            log.err('No build files, cannot generate DFU files')
            return
        else:
            log.inf('Creating DFU files')

        res = os.system('adafruit-nrfutil dfu genpkg --application ' + hex_path + ' ' + dest_path + ' --dev-type 0x0052 --dev-revision 52840 --sd-req 0x0123')

        with zipfile.ZipFile(dest_path, 'r') as zip_ref:
            zip_ref.extractall(os.path.join(build_path, 'zephyr_ada_bl'))

        if platform.system() == 'Windows':
            os.system('move ' + os.path.join(build_path, 'zephyr_ada_bl', 'zephyr.dat') + ' ' + os.path.join(build_path, 'zephyr.dat'))
            os.system('rmdir /s /q ' + os.path.join(build_path, 'zephyr_ada_bl'))
            os.system('del ' + dest_path)
        elif platform.system() == 'linux':
            os.system('cp -v' + os.path.join(build_path, 'zephyr_ada_bl', 'zephyr.dat') + ' ' + os.path.join(build_path, 'zephyr.dat'))
            os.system('rm -r -v' + os.path.join(build_path, 'zephyr_ada_bl'))
            os.system('rm -v' + dest_path)
        else:
            log.err('Platform not supported: ' platform.system())

        return
