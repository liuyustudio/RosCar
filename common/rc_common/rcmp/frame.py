# -*- coding: utf-8 -*-
'''
frame.py

ROS Car Msg Protocol(RCMP) Frame Encoding, Decoding Module.

Copyright:  All Rights Reserved 2019-2019

History:
    1   2019-07-14  Liu Yu (source@liuyu.com)   Initial Version
'''

from struct import Struct

import rc_common.errno as ERR
import rc_common.exception as EXP
import rc_common.rcmp.const as CONST

FRAME_HEAD_FMT = Struct("!"  # Network endian
                        "B"  # 1 Byte, Start Flag, always 0x5A('Z')
                        "B"  # 1 Byte, Reverse
                        "H"  # 2 Bytes, Payload size
                        )


class RCMPFrame(object):
    @staticmethod
    def encode(payload):
        ''' Encoding RCMP Frame

        Args:
            payload: payload('str' or 'utf-8 bytes' format)
        Returns:
            RCMP Frame
        Raises:
            EXP.VRException: An error occurred when encoding.
        '''

        if isinstance(payload, str):
            raw_bytes = bytes(payload, 'utf-8')
        elif isinstance(payload, bytes):
            raw_bytes = payload
        elif not payload:
            raw_bytes = bytes()
        else:
            raise EXP.RCException(
                errno=ERR.ERROR,
                errmsg='invalid payload type: {}'.format(type(payload)))

        payloadSize = len(raw_bytes)

        if payloadSize > CONST.RCMP_MAXPAYLOAD:
            raise EXP.RCException(ERR.ERROR_INVALID,
                                  'payload too large')

        frame_head = FRAME_HEAD_FMT.pack(CONST.RCMP_STARTFLAG,
                                         CONST.RCMP_VERSION, payloadSize)

        return frame_head + raw_bytes

    @staticmethod
    def parse(rawbuf):
        ''' Parse RCMP Frame

        Args:
            param rawbuf: raw buffer

        Returns:
            (raw_payload, frame_length)

        Raises:
            EXP.VRException: An error occurred when parse raw data.
            EXP.RCException_More_Data: Need more data
        '''

        bufsize = len(rawbuf)

        if bufsize < CONST.RCMP_FRAMEHEADSIZE:
            # Need more data
            raise EXP.RCException_More_Data(ERR.ERROR,
                                            'need more data')

        # check start flag
        if rawbuf[0] != CONST.RCMP_STARTFLAG:
            raise EXP.RCException(ERR.ERROR_INVALID,
                                  'invalid Start Flag')

        # parse buffer
        [_, version, payloadSize] = FRAME_HEAD_FMT.unpack(
            rawbuf[:CONST.RCMP_FRAMEHEADSIZE])

        # check version
        if version != CONST.RCMP_VERSION:
            raise EXP.RCException(ERR.ERROR_UNSUPPORT,
                                  'unsupport version: {}'.format(version))

        # check payload size
        if payloadSize > CONST.RCMP_MAXPAYLOAD:
            raise EXP.RCException(ERR.ERROR_INVALID,
                                  'payload too large')

        # check buffer size
        if CONST.RCMP_FRAMEHEADSIZE + payloadSize > bufsize:
            # need more data
            raise EXP.RCException_More_Data(ERR.ERROR,
                                            'need more data')

        frame_length = CONST.RCMP_FRAMEHEADSIZE + payloadSize
        if payloadSize > 0:
            raw_payload = rawbuf[CONST.RCMP_FRAMEHEADSIZE:frame_length]
        else:
            raw_payload = None

        return (raw_payload, frame_length)
