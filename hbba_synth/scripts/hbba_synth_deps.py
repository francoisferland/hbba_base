#!/usr/bin/env python

import re
import yaml
import sys
import subprocess
import os

from roslib.packages import find_resource

def parse(fn):
    try:
        lines = subprocess.check_output("grep include {0}".format(fn), shell=True)
    except:
        return

    for l in lines.split('\n'):
        v = yaml.load(l)
        if (v is None):
            continue
        res = []
        pkg = v[0]['include']['pkg']
        src = v[0]['include']['file']
        try:
            res = find_resource(pkg,src)
        except:
            base_dir = os.path.dirname(fn)
            for root, dirs, files in os.walk(base_dir):
                if src in files:
                    res.append(os.path.join(root,src))

        if (len(res) == 0):
            exit(-1)

        path = res[0]
        print path 
        parse(path)

if len(sys.argv) < 2:
    print "Usage: {0} source_file".format(sys.argv[0])

fn = sys.argv[1]
parse(fn)
