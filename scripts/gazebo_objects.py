#!/usr/bin/env python

import time

from utils import *
from list_models import *

def add_object(name, model, pose):
    d = get_gazebo_models()
    if model in d.keys():
        sdf = os.path.join(d[model]['path'], d[model]['sdf'][0])        
        object_pose = coords2pose(pose)
        spawn_object(sdf, name, object_pose)
    else:
        print("Model %s unknown" %model)

def add_objects(infile):
    with open(infile,'r') as f:
        l = f.readline()
        while l!='' and l[0:4] != '#END':
            l = l.strip()
            if len(l)>1 and l[0]!='#':
                v = l.split()
                if len(v)>7:
                    try:
                        name = v[0]
                        model = v[1]
                        pose = [ float(x) for x in v[2:] ]
                        add_object(name, model, pose)
                        time.sleep(0.2)
                    except Exception as e:
                        print("%s%s\n" %(l,e))
                else:
                    print("Parse error: %s" %l)
            l = f.readline()
        f.close()


def del_objects(infile):
    with open(infile,'r') as f:
        l = f.readline()
        while l!=''  and l[0:4] != '#END':
            l = l.strip()
            if len(l)>1 and l[0]!='#':
                v = l.split()
                delete_object(v[0])
            l = f.readline()
        f.close()



if __name__ == '__main__':
    
    if len(sys.argv)==1 or sys.argv[1]=='-h':
            print("Options")
            print("-h\thelp")
            print("-l\tlist objects")
            print("-m\tlist available models")
            print("-d <obj>|<filename>\tdelete an object or all objects in config file")
            print("-a <obj>  <type> <x> <y> <z> <yaw> <pitch> <roll>|<filename>\tadd an object or all objects in config file")

    else:
        if sys.argv[1]=='-m':
            print_models()

        elif sys.argv[1]=='-l':
            l = list_object_names()
            for obj in l:
                p = get_object_pose(obj)
                #s = get_model_state(obj)
                print("%s  %s" %(obj,pose_str(p)))

        elif sys.argv[1]=='-a' and len(sys.argv)>2:
            name = sys.argv[2]
            if os.path.isfile(name):
                add_objects(name)
            elif len(sys.argv)>9:
                model = sys.argv[3]
                pose = [ float(x) for x in sys.argv[4:] ]
                add_object(name, model, pose)

        elif sys.argv[1]=='-d' and len(sys.argv)>2:
            name = sys.argv[2]
            if os.path.isfile(name):
                del_objects(name)
            else:
                delete_object(name)



