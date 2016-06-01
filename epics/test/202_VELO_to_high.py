#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
# http://cars9.uchicago.edu/software/python/pyepics3/devices.html#module-motor
#
# m-epics-singlemotion/src/main/test/singlemotion_test.py
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import math
import time
###

polltime = 0.2

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def waitForStart(motor, tc_no, wait_for_start):
    while wait_for_start > 0:
        wait_for_start -= polltime
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_start, dmov, movn, motor.get_position(readback=True,dial=True))
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for_start -= polltime
    return False

def waitForStop(motor, tc_no, wait_for_stop):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        dmov = int(motor.get('DMOV'))
        movn = int(motor.get('MOVN'))
        print '%s: wait_for_stop=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_stop, dmov, movn, motor.get_position(readback=True,dial=True))
        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False

def printMSTA(tc_no, tc_info, motor, pv_MSTA):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_MOVING   = 1 << (11 -1)
    MSTA_BIT_PROBLEM  = 1 << (10 -1)
    MSTA_BIT_FOLLOW_ERR  = 1 << (7 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    msta = int(pv_MSTA.get(use_monitor=False))

    homed   = bool(msta & MSTA_BIT_HOMED)
    lls     = bool(msta & MSTA_BIT_MINUS_LS)
    moving  = bool(msta & MSTA_BIT_MOVING)
    problem = bool(msta & MSTA_BIT_PROBLEM)
    follow  = bool(msta & MSTA_BIT_FOLLOW_ERR)
    hls     = bool(msta & MSTA_BIT_PLUS_LS)
    dmov = int(motor.get('DMOV'))
 
    print '%s %s dmov=%d homed=%d HLS=%d moving=%d problem=%d follow=%d LLS=%d' % (tc_no, tc_info, dmov, homed, lls, moving, problem, follow, hls    )


class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)

    MSTA_BIT_MOVING   = 1 << (11 -1)
    MSTA_BIT_PROBLEM  = 1 << (10 -1)
    MSTA_BIT_FOLLOW_ERR  = 1 << (7 -1)

    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    motm1   = epics.Motor(os.getenv("TESTEDMOTORAXIS"))
    pvm1 = epics.PV(os.getenv("TESTEDMOTORAXIS"))
    pv_Err   = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-Err")
    pv_nErrorId = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrId")
    pv_nErrRst = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrRst")
    pv_MSTA = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".MSTA")
    pv_VELO = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".VELO")

    saved_HLM = motm1.get('HLM')
    saved_LLM = motm1.get('LLM')
    
    def setUp(self):
        print 'set up'


    def tearDown(self):
        print 'clean up'
    
    # 10% Position
    def test_TC_202(self):
        tc_no = "TC-202-10-percent-Position"
        print '%s' % tc_no
        val =  (self.saved_HLM + 9 * self.saved_LLM) / 10
        ret = self.motm1.move(dval, dial=False, wait=True)
        assert (ret == 0)
        rbv = self.motm1.get_position(readback=True,dial=False)
        print '%s val=%f rbv=%f' % (tc_no, val, rbv)
        assert calcAlmostEqual(self.motm1, tc_no, val, rbv, 2)
        
    # Try to move to 90% Position with too high velocity
    def test_TC_202(self):
        tc_no = "TC-202-VELO_to_high"
        saved_VELO = self.pv_VELO.get(use_monitor=False)

        self.pv_VELO.put(1000)
        val =  (9 * self.saved_HLM + self.saved_LLM) / 10
        ret = self.motm1.move(val, dial=False, wait=True)

        self.motm1.put('VAL', val)
        printMSTA(tc_no, "after put", self.motm1, self.pv_MSTA)
        ret = waitForStart(self.motm1, tc_no, 2.0)
        # dummy wait
        printMSTA(tc_no, "after waitForStart", self.motm1, self.pv_MSTA)

        ret = waitForStop(self.motm1, tc_no, 4.0)
        self.assertEqual(True, ret, 'waitForStop return True')

        printMSTA(tc_no, "after waitForStop", self.motm1, self.pv_MSTA)
        msta = int(self.pv_MSTA.get(use_monitor=False))
        printMSTA(tc_no, "after get msta", self.motm1, self.pv_MSTA)
        self.pv_VELO.put(saved_VELO)

        self.assertNotEqual(0, msta & self.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, msta & self.MSTA_BIT_FOLLOW_ERR, 'Error MSTA.Following Error should not be set)')
        self.assertEqual(0, msta & self.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        bError   = self.pv_Err.get(use_monitor=False)
        nErrorId = self.pv_nErrorId.get(use_monitor=False)
        print '%s Error bError=%d nErrorId=%d' % (tc_no, bError, nErrorId)

        self.assertNotEqual(0, bError,   'bError')
        self.assertNotEqual(0, nErrorId, 'nErrorId')
        
        self.pv_nErrRst.put(1)

