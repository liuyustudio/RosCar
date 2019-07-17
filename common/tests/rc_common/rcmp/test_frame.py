#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
test_frame.py

Unit test case for rc_common.rcmp.frame.

Copyright:  All Rights Reserved 2019-2019

History:
    1   2019-07-14  Liu Yu (source@liuyu.com)   Initial Version
'''

import json
import unittest

from rc_common import errno as ERR
from rc_common import exception as EXP
from rc_common.facility import ordered
from rc_common.rcmp import const as CONST
from rc_common.rcmp.frame import FRAME_HEAD_FMT, RCMPFrame

PAYLOAD = {"msg": "Login", "seq": 0,
           "payload": {"ver": 0, "type": "", "id": ""}}
PAYLOAD_STR = json.dumps(PAYLOAD)
PAYLOAD_BYTES = bytes(PAYLOAD_STR, 'utf-8')
FRAME_HEAD = FRAME_HEAD_FMT.pack(CONST.RCMP_STARTFLAG,
                                 CONST.RCMP_VERSION,
                                 len(PAYLOAD_BYTES))
FRAME = FRAME_HEAD + PAYLOAD_BYTES

FRAME_NO_PAYLOAD = FRAME_HEAD_FMT.pack(CONST.RCMP_STARTFLAG,
                                       CONST.RCMP_VERSION,
                                       0)


class TestRCMPFrame_encode(unittest.TestCase):

    def test_encode(self):
        frame = RCMPFrame.encode(PAYLOAD_STR)
        self.assertEqual(frame, FRAME)

    def test_encode_bytes(self):
        frame = RCMPFrame.encode(PAYLOAD_BYTES)
        self.assertEqual(frame, FRAME)

    def test_encode_no_payload(self):
        frame = RCMPFrame.encode('')
        self.assertEqual(frame, FRAME_NO_PAYLOAD)

    def test_encode_no_payload_bytes(self):
        frame = RCMPFrame.encode(None)
        self.assertEqual(frame, FRAME_NO_PAYLOAD)

    def test_raise_exception_invalid_payload_type(self):
        with self.assertRaises(EXP.RCException) as exp:
            RCMPFrame.encode(1234)
        self.assertEqual(ERR.ERROR, exp.exception.errno)
        self.assertTrue('invalid payload type: ' in exp.exception.errmsg)

        with self.assertRaises(EXP.RCException) as exp:
            RCMPFrame.encode(PAYLOAD)
        self.assertEqual(ERR.ERROR, exp.exception.errno)
        self.assertTrue('invalid payload type: ' in exp.exception.errmsg)

    def test_raise_exception_payload_too_large(self):

        large_payload = 'a' * (0x10000 - CONST.RCMP_FRAMEHEADSIZE)
        frame = RCMPFrame.encode(large_payload)
        self.assertIsNotNone(frame)

        large_payload += 'a'
        with self.assertRaises(EXP.RCException) as exp:
            RCMPFrame.encode(large_payload)
        self.assertEqual(ERR.ERROR_INVALID, exp.exception.errno)
        self.assertEqual('payload too large', exp.exception.errmsg)


class TestRCMPFrame_parse(unittest.TestCase):

    def test_parse(self):
        payload_str, frame_length = RCMPFrame.parse(FRAME)
        payload = json.loads(payload_str)

        self.assertEqual(CONST.RCMP_FRAMEHEADSIZE +
                         len(PAYLOAD_STR), frame_length)
        self.assertEqual(ordered(PAYLOAD), ordered(payload))

    def test_parse_no_payload(self):
        payload_str, frame_length = RCMPFrame.parse(FRAME_NO_PAYLOAD)
        self.assertEqual(CONST.RCMP_FRAMEHEADSIZE, frame_length)
        self.assertEqual(None, payload_str)

    def test_raise_exception_need_more_data_wait_head(self):

        rawbuf = bytes('abc', 'utf-8')
        with self.assertRaises(EXP.RCException_More_Data) as exp:
            RCMPFrame.parse(rawbuf)
        self.assertEqual(ERR.ERROR, exp.exception.errno)
        self.assertEqual('need more data', exp.exception.errmsg)

    def test_raise_exception_need_more_data_wait_payload(self):

        rawbuf = FRAME_HEAD_FMT.pack(CONST.RCMP_STARTFLAG,
                                     CONST.RCMP_VERSION,
                                     1)
        with self.assertRaises(EXP.RCException_More_Data) as exp:
            RCMPFrame.parse(rawbuf)
        self.assertEqual(ERR.ERROR, exp.exception.errno)
        self.assertEqual('need more data', exp.exception.errmsg)

    def test_raise_exception_invalid_start_flag(self):

        rawbuf = FRAME_HEAD_FMT.pack(0x00,
                                     CONST.RCMP_VERSION,
                                     0)
        with self.assertRaises(EXP.RCException) as exp:
            RCMPFrame.parse(rawbuf)
        self.assertEqual(ERR.ERROR_INVALID, exp.exception.errno)
        self.assertEqual('invalid Start Flag', exp.exception.errmsg)

    def test_raise_exception_invalid_version(self):

        rawbuf = FRAME_HEAD_FMT.pack(CONST.RCMP_STARTFLAG,
                                     0xFF,
                                     0)
        with self.assertRaises(EXP.RCException) as exp:
            RCMPFrame.parse(rawbuf)
        self.assertEqual(ERR.ERROR_UNSUPPORT, exp.exception.errno)
        self.assertTrue('unsupport version: ' in exp.exception.errmsg)

    def test_raise_exception_payload_too_large(self):

        rawbuf = FRAME_HEAD_FMT.pack(CONST.RCMP_STARTFLAG,
                                     CONST.RCMP_VERSION,
                                     0x10000 - CONST.RCMP_FRAMEHEADSIZE + 1)
        with self.assertRaises(EXP.RCException) as exp:
            RCMPFrame.parse(rawbuf)
        self.assertEqual(ERR.ERROR_INVALID, exp.exception.errno)
        self.assertEqual('payload too large', exp.exception.errmsg)


class TestRCMPFrame(unittest.TestCase):

    def test_encode_and_parse(self):
        frame = RCMPFrame.encode(PAYLOAD_STR)
        payload_str, frame_length = RCMPFrame.parse(frame)
        payloadObj = json.loads(payload_str)

        self.assertEqual(CONST.RCMP_FRAMEHEADSIZE +
                         len(PAYLOAD_STR), frame_length)
        self.assertEqual(ordered(PAYLOAD), ordered(payloadObj))

    def test_encode_and_parse_no_payload(self):
        frame = RCMPFrame.encode(None)
        payload_str, frame_length = RCMPFrame.parse(frame)

        self.assertEqual(CONST.RCMP_FRAMEHEADSIZE, frame_length)
        self.assertEqual(None, payload_str)


if __name__ == '__main__':
    unittest.main()
