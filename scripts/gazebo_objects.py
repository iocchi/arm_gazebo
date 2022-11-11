#!/usr/bin/env python

import time

from utils import *
from list_models import *


if __name__ == '__main__':
    
    if len(sys.argv)>1:

        if sys.argv[1]=='-m':
            print_models()
        elif sys.argv[1]=='-l':
            l = list_object_names()
            for obj in l:
                p = get_object_pose(obj)
                #s = get_model_state(obj)
                print("%s  %s" %(obj,pose_str(p)))
        elif sys.argv[1]=='-d' and len(sys.argv)>2:
            name = sys.argv[2]
            if os.path.isfile(name):
                with open(name,'r') as f:
                    l = f.readline()
                    while l!=''  and l[0:4] != '#END':
                        l = l.strip()
                        if len(l)>1 and l[0]!='#':
                            v = l.split()
                            delete_object(v[0])
                        l = f.readline()
                    f.close()
            else:
                delete_object(name)
        elif sys.argv[1]=='-a' and len(sys.argv)>9:
            name = sys.argv[2]
            model = sys.argv[3]
            d = get_gazebo_models()
            if model in d.keys():
                sdf = os.path.join(d[model]['path'], d[model]['sdf'][0])
                pose = [ float(x) for x in sys.argv[4:] ]
                object_pose = coords2pose(pose)
                spawn_object(sdf, name, object_pose)
            else:
                print("Model %s unknown" %model)
        elif sys.argv[1]=='-i' and len(sys.argv)>2:
            infile = sys.argv[2]
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
                                d = get_gazebo_models()
                                if model in d.keys():
                                    sdf = os.path.join(d[model]['path'], d[model]['sdf'][0])
                                    pose = [ float(x) for x in v[2:] ]
                                    object_pose = coords2pose(pose)
                                    spawn_object(sdf, name, object_pose)
                                    time.sleep(0.2)

                                else:
                                    print("Model %s unknown" %model)
                            except Exception as e:
                                print("%s%s\n" %(l,e))
                        else:
                            print("Parse error: %s" %l)
                    l = f.readline()
                f.close()


