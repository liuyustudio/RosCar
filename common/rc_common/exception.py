#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
exception.py

Exceptions for ROS Car project

Copyright:  All Rights Reserved 2019-2019

History:
    1   2019-07-14  Liu Yu (source@liuyu.com)   Initial Version
'''

from . import errno as ERR


class RCException(Exception):
    ''' ROS Car Root Exception '''

    def __init__(self, errno=ERR.ERROR, errmsg="", payload=None):
        self.errmsg = errmsg
        self.errno = errno
        self.payload = payload

    def __str__(self):
        return '{}: {}'.format(self.errno, self.errmsg)


class RCException_More_Data(RCException):
    ''' ROS Car Exception: Need more data '''
