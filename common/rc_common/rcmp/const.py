# -*- coding: utf-8 -*-
'''
const.py

Constant Variables Definition.

Copyright:  All Rights Reserved 2019-2019

History:
    1   2019-07-14  Liu Yu (source@liuyu.com)   Initial Version
'''

RCMP_FRAMEHEADSIZE = 4
RCMP_STARTFLAG = 0x5A
RCMP_VERSION = 1
RCMP_MIN_SIGNALING_LENGTH = RCMP_FRAMEHEADSIZE
RCMP_MAXPAYLOAD = 0x10000 - RCMP_FRAMEHEADSIZE  # almost 16 KB


SIG_LOGIN = 'Login'
SIG_LOGIN_RESP = 'LoginResp'
SIG_LOGOUT = 'Logout'
SIG_LOGOUT_RESP = 'LogoutResp'
SIG_PING = 'Ping'
SIG_PONG = 'Pong'
SIG_CONTROL = 'Ctl'
SIG_CONTROL_RESP = 'CtlResp'
SIG_MT = 'Mt'
SIG_MT_RESP = 'MtResp'
SIG_REPORT = 'Report'
SIG_REPORT_RESP = 'ReportResp'


SIG_LIST = [
    SIG_LOGIN,
    SIG_LOGIN_RESP,
    SIG_LOGOUT,
    SIG_LOGOUT_RESP,
    SIG_PING,
    SIG_PONG,
    SIG_CONTROL,
    SIG_CONTROL_RESP,
    SIG_MT,
    SIG_MT_RESP,
    SIG_REPORT,
    SIG_REPORT_RESP,
]


RESP_DICT = {
    SIG_LOGIN: SIG_LOGIN_RESP,
    SIG_LOGOUT: SIG_LOGOUT_RESP,
    SIG_PING: SIG_PONG,
    SIG_CONTROL: SIG_CONTROL_RESP,
    SIG_MT: SIG_MT_RESP,
    SIG_REPORT: SIG_REPORT_RESP,
}


RCMP_WAITING_SIGNALING_COUNT = 64
RCMP_HEARTBEAT_INTERVAL = 30
RCMP_SESSION_TIMEOUT = RCMP_HEARTBEAT_INTERVAL * 3


DEV_TYPE_UNSET = ''
DEV_TYPE_TERM = 'Terminal'
DEV_TYPE_CTRL = 'Client'
DEV_TYPE_LIST = [
    DEV_TYPE_TERM,
    DEV_TYPE_CTRL
]


STATISTIC_INTERVAL = 60
LOGIN_TIMEOUT = 15
HEARTBIT_TIMEOUT = 30
HEARTBIT_ALIVE_COUNT = 2
