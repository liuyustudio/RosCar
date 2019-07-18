#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
test_signaling.py

Unit test case for rc_common.rcmp.signaling.

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
from rc_common.rcmp.frame import FRAME_HEAD_FMT  # , RCMPFrame
from rc_common.rcmp.signaling import RCMPSig


class TestRCMPSig_req(unittest.TestCase):

    def test_generate_req(self):

        PAYLOAD = {"ver": 0,
                   "type": CONST.DEV_TYPE_TERM,
                   "id": "abcdefghijklmnopqrstuvwxyz"
                   }
        REQ = {'cmd': CONST.SIG_LOGIN,
               'seq': 123456,
               'payload': PAYLOAD
               }

        req = RCMPSig.req(CONST.SIG_LOGIN,
                          123456,
                          PAYLOAD)

        self.assertEqual(ordered(REQ), ordered(req))

    def test_valid_cmd_id(self):
        for cmd in CONST.SIG_REQ_CMD_LIST:
            RCMPSig.req(cmd,
                        123456,
                        None)

    def test_raise_exception_invalid_cmd_id(self):
        for cmd in CONST.SIG_RESP_CMD_LIST:
            with self.assertRaises(EXP.RCException) as exp:
                RCMPSig.req(cmd,
                            123456,
                            "payload of req")
            self.assertEqual(ERR.ERROR_INVALID, exp.exception.errno)
            self.assertTrue('invalid req cmd id: ' in exp.exception.errmsg)

    def test_raise_exception_invalid_cmd_id_2(self):
        for cmd in CONST.SIG_CMD_LIST:
            try:
                RCMPSig.req(cmd,
                            123456,
                            "payload of req")
                self.assertTrue(cmd in CONST.SIG_REQ_CMD_LIST)
            except EXP.RCException as e:
                self.assertTrue(cmd in CONST.SIG_RESP_CMD_LIST)
                self.assertEqual(ERR.ERROR_INVALID, e.errno)
                self.assertTrue('invalid req cmd id: ' in e.errmsg)


class TestRCMPSig_resp(unittest.TestCase):

    def test_generate_resp(self):

        RESP = {
            'cmd': CONST.SIG_LOGIN_RESP,
            'seq': 123456,
            'errno': ERR.FAIL,
            'errmsg': "login fail",
            'payload': "payload of login fail"
        }

        resp = RCMPSig.resp(CONST.SIG_LOGIN_RESP,
                            123456,
                            ERR.FAIL,
                            "login fail",
                            "payload of login fail")

        self.assertEqual(ordered(RESP), ordered(resp))

    def test_generate_resp_from_sig(self):

        REQ = {
            'cmd': CONST.SIG_LOGIN,
            'seq': 123456,
            'payload': None
        }

        RESP = {
            'cmd': CONST.SIG_LOGIN_RESP,
            'seq': 123456,
            'errno': ERR.FAIL,
            'errmsg': "login fail",
            'payload': "payload of login fail"
        }

        resp = RCMPSig.resp_from_sig(REQ, ERR.FAIL, "login fail",
                                     "payload of login fail")

        self.assertEqual(ordered(RESP), ordered(resp))

    def test_valid_cmd_id(self):
        for cmd in CONST.SIG_RESP_CMD_LIST:
            RCMPSig.resp(cmd,
                         123456,
                         ERR.FAIL,
                         "fail",
                         "payload of resp")

    def test_raise_exception_invalid_cmd_id(self):
        for cmd in CONST.SIG_REQ_CMD_LIST:
            with self.assertRaises(EXP.RCException) as exp:
                RCMPSig.resp(cmd,
                             123456,
                             ERR.FAIL,
                             "fail",
                             "payload of resp")
            self.assertEqual(ERR.ERROR_INVALID, exp.exception.errno)
            self.assertTrue('invalid resp cmd id: ' in exp.exception.errmsg)

    def test_raise_exception_invalid_cmd_id_2(self):
        for cmd in CONST.SIG_CMD_LIST:
            try:
                RCMPSig.resp(cmd,
                             123456,
                             ERR.FAIL,
                             "fail",
                             "payload of resp")
                self.assertTrue(cmd in CONST.SIG_RESP_CMD_LIST)
            except EXP.RCException as e:
                self.assertTrue(cmd in CONST.SIG_REQ_CMD_LIST)
                self.assertEqual(ERR.ERROR_INVALID, e.errno)
                self.assertTrue('invalid resp cmd id: ' in e.errmsg)


class TestRCMPSig(unittest.TestCase):

    def test_encode_and_parse(self):
        SIG = {
            "cmd": "Login",
            "seq": 0,
            "payload": {
                "ver": 0,
                "type": CONST.DEV_TYPE_CTRL,
                "id": "abcdefghijklmnopqrstuvwxyz"
            }
        }

        frame = RCMPSig.encode(SIG)
        sig, frame_length = RCMPSig.parse(frame)

        self.assertEqual(CONST.RCMP_FRAMEHEADSIZE +
                         len(bytes(json.dumps(SIG), 'utf-8')), frame_length)
        self.assertEqual(ordered(SIG), ordered(sig))


if __name__ == '__main__':
    unittest.main()
