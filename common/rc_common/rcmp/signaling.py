# -*- coding: utf-8 -*-
'''
signaling.py

ROS Car Msg Protocol signoaling encoding and parser module

Copyright:  All Rights Reserved 2019-2019

History:
    1   2019-07-14  Liu Yu (source@liuyu.com)   Initial Version
'''

import json

from rc_common import errno as ERR
from rc_common import exception as EXP
from rc_common.rcmp import const as CONST
from .frame import RCMPFrame


class RCMPSig(object):
    @staticmethod
    def parse(rawbuf):
        ''' parse signaling from raw data

        Args:
            rawbuf: buffer

        Returns:
            (sig, frame_length)

        Raises:
            EXP.RCException: An error occurred when parse raw data.
            EXP.RCException_More_Data: Need more data
        '''

        # parse frame payload from raw data
        frame_payload, frame_length = RCMPFrame.parse(rawbuf)

        # parse frame payload

        try:
            if isinstance(frame_payload, bytes):
                # decode buffer as 'utf-8'
                frame_payload = frame_payload.decode('utf-8')
            sig = json.loads(frame_payload)

            cmd = sig['cmd']
            if cmd not in CONST.SIG_LIST:
                raise EXP.RCException(ERR.ERROR_UNSUPPORT,
                                      'unsupport cmd: {}'.format(cmd))
        except Exception as e:
            raise EXP.RCException(ERR.ERROR_INVALID,
                                  'parse frame paload fail: {}'.format(e))

        return (sig, frame_length)

    @staticmethod
    def resp(sig, errno, errmsg, payload=None):
        cmd = sig['cmd']
        seq = sig['seq']

        resp_cmd = CONST.RESP_DICT[cmd]

        return {
            'msg': resp_cmd,
            'seq': seq,
            'errno': errno,
            'errmsg': errmsg,
            'payload': payload
        }

    @staticmethod
    def resp_err(respCmd, seq, errno, errmsg, payload=None):
        return {
            'msg': respCmd,
            'seq': seq,
            'errno': errno,
            'errmsg': errmsg,
            'payload': payload
        }

    @staticmethod
    def resp_exp(respCmd, seq, exp):
        if isinstance(exp, EXP.RCException):
            RCMPSig.resp_err(respCmd, seq,
                             exp.errno, exp.errmsg,
                             exp.payload)
        else:
            RCMPSig.resp_err(respCmd, seq,
                             ERR.ERROR, '{}'.format(exp),
                             None)

    @staticmethod
    def encode(self, sig):
        ''' encode signaling into RCMP Frame '''

        return RCMPFrame.encode(json.dumps(sig))
