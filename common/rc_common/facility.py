# -*- coding: utf-8 -*-
'''
facility.py

Facility Module.

Copyright:  All Rights Reserved 2019-2019

History:
    1   2019-07-14  Liu Yu (source@liuyu.com)   Initial Version
'''

def ordered(obj):
    ''' from: https://stackoverflow.com/a/25851972 '''

    if isinstance(obj, dict):
        return sorted((k, ordered(v)) for k, v in obj.items())
    if isinstance(obj, list):
        return sorted(ordered(x) for x in obj)
    else:
        return obj
