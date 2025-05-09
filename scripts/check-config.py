#! /usr/bin/env python3

# Copyright (c) 2015, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of The Linux Foundation nor
#       the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Android kernel configuration validator.

The Android kernel reference trees contain some config stubs of
configuration options that are required for Android to function
correctly, and additional ones that are recommended.

This script can help compare these base configs with the ".config"
output of the compiler to determine if the proper configs are defined.
"""

from collections import namedtuple
from optparse import OptionParser
import re
import sys

version = "check-config.py, version 0.0.1"

req_re = re.compile(r'''^CONFIG_(.*)=(.*)$''')
forb_re = re.compile(r'''^# CONFIG_(.*) is not set$''')
comment_re = re.compile(r'''^(#.*|)$''')

Enabled = namedtuple('Enabled', ['name', 'value'])
Disabled = namedtuple('Disabled', ['name'])

def walk_config(name):
    with open(name, 'r') as fd:
        for line in fd:
            line = line.rstrip()
            m = req_re.match(line)
            if m:
                yield Enabled(m.group(1), m.group(2))
                continue

            m = forb_re.match(line)
            if m:
                yield Disabled(m.group(1))
                continue

            m = comment_re.match(line)
            if m:
                continue

            print("WARNING: Unknown .config line: ", line)

class Checker():
    def __init__(self):
        self.required = {}
        self.exempted = set()
        self.forbidden = set()

    def add_required(self, fname):
        for ent in walk_config(fname):
            if type(ent) is Enabled:
                self.required[ent.name] = ent.value
            elif type(ent) is Disabled:
                if ent.name in self.required:
                    del self.required[ent.name]
                self.forbidden.add(ent.name)

    def add_exempted(self, fname):
        with open(fname, 'r') as fd:
            for line in fd:
                line = line.rstrip()
                self.exempted.add(line)

    def check(self, path):
        failure = False

        # Don't run this for mdm targets
        if re.search('mdm', path):
            print("Not applicable to mdm targets... bypassing")
        else:
            for ent in walk_config(path):
                # Go to the next iteration if this config is exempt
                if ent.name in self.exempted:
                   continue

                if type(ent) is Enabled:
                    if ent.name in self.forbidden:
                        print("error: Config should not be present: %s" %ent.name)
                        failure = True

                    if ent.name in self.required and ent.value != self.required[ent.name]:
                        print("error: Config has wrong value: %s %s expecting: %s" \
                                 %(ent.name, ent.value, self.required[ent.name]))
                        failure = True

                elif type(ent) is Disabled:
                    if ent.name in self.required:
                        print("error: Config should be present, but is disabled: %s" %ent.name)
                        failure = True

        if failure:
            sys.exit(1)

def main():
    usage = """%prog [options] path/to/.config"""
    parser = OptionParser(usage=usage, version=version)
    parser.add_option('-r', '--required', dest="required",
            action="append")
    parser.add_option('-e', '--exempted', dest="exempted",
            action="append")
    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error("Expecting a single path argument to .config")
    elif options.required is None or options.exempted is None:
        parser.error("Expecting a file containing required configurations")

    ch = Checker()
    for r in options.required:
        ch.add_required(r)
    for e in options.exempted:
        ch.add_exempted(e)

    ch.check(args[0])

if __name__ == '__main__':
    main()
