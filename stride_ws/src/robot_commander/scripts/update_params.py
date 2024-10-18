#!/usr/bin/env python
import yaml
import os
import io

cur_dir = os.path.dirname(os.path.abspath(__file__))
param_file = os.path.abspath( os.path.join(cur_dir, '..', '..','..','params', 'params.yaml'))
param_file_original = os.path.abspath( os.path.join(cur_dir, '..', '..','..','params', 'params.yaml.original'))

def open_param_file():
    p = None
    try:
        with open(param_file, 'r') as stream:
            try:
                p = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    except Exception as e:
        print(e)
    return p

def update_param_reverse_speed(speed):
    p = open_param_file()
    if p == None:
        return False
    
    try:
        with io.open(param_file, 'w', encoding='utf8') as outfile:
            p['reverse_speed_goal'] = speed
            yaml.dump(p, outfile, default_flow_style=False, allow_unicode=True)
    except Exception as e:
        print(e)
        return False
    return True

def update_param_reverse_rate(rate):
    p = open_param_file()
    if p == None:
        return False
    
    try:
        with io.open(param_file, 'w', encoding='utf8') as outfile:
            p['reverse_speed_rate'] = rate
            yaml.dump(p, outfile, default_flow_style=False, allow_unicode=True)
    except Exception as e:
        print(e)
        return False
    return True

def restore_original_parameters():
    try:
        with open(param_file_original, 'r') as f:
            s = f.read()
    except:
        return False

    try:
        with open(param_file, 'w') as f:
            f.write(s)
    except:
        return False
    return True

    

