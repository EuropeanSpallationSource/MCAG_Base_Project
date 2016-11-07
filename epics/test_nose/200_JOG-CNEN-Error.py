#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
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
        dmov = int(epics.caget(motor + '.DMOV', use_monitor=False))
        movn = int(epics.caget(motor + '.MOVN', use_monitor=False))
        rbv = epics.caget(motor + '.RBV')
        print '%s: wait_for_start=%f dmov=%d movn=%d rbv=%f' % (tc_no, wait_for_start, dmov, movn, rbv)
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for_start -= polltime
    return False

def waitForStop(motor, tc_no, wait_for_stop):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        dmov = int(epics.caget(motor + '.DMOV', use_monitor=False))
        movn = int(epics.caget(motor + '.MOVN', use_monitor=False))
        rbv = epics.caget(motor + '.RBV', use_monitor=False)
        print '%s: wait_for_stop=%f dmov=%d movn=%d rbv=%f' % (tc_no, wait_for_stop, dmov, movn, rbv)
        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False

class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)

    MSTA_BIT_MOVING   = 1 << (11 -1)
    MSTA_BIT_PROBLEM  = 1 << (10 -1)
    MSTA_BIT_FOLLOW_ERR  = 1 << (7 -1)

    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = os.getenv("TESTEDMOTORAXIS")
    #motm1   = epics.Motor(os.getenv("TESTEDMOTORAXIS"))
    pvm1 = epics.PV(os.getenv("TESTEDMOTORAXIS"))
    pv_Err   = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-Err")
    pv_nErrorId = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrId")
    pv_nErrRst = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrRst")
    pv_MSTA = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".MSTA")
    
    
    saved_HLM = epics.caget(m1 + '.HLM')
    saved_LLM = epics.caget(m1 + '.LLM')
    saved_CNEN = epics.caget(m1 + '.CNEN')
    
    def setUp(self):
        print 'set up'


    def tearDown(self):
        print 'clean up'

    
    # 10% UserPosition
    def test_TC_201(self):
        tc_no = "TC-201-10-percent-dialPosition"
        print '%s' % tc_no
        epics.caput(self.m1 + '.CNEN', 1)
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        epics.caput(self.m1 + '.VAL', destination, wait=True)

        
    # Jog, wait for start, power off, check error, reset error
    def test_TC_202(self):
        tc_no = "TC-202-JOG-_CNEN"
        epics.caput(self.m1 + '.CNEN', 0)
        epics.caput(self.m1 + '.JOGF', 1)
        ret = waitForStart(self.m1, tc_no, 2.0)
        # dummy wait

        ret = waitForStop(self.m1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStop return True')
       
        msta = int(epics.caget(self.m1 + '.MSTA', use_monitor=False))
        print '%s Error msta=%x' % (tc_no, msta)
        self.assertNotEqual(0, msta & self.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, msta & self.MSTA_BIT_FOLLOW_ERR, 'Error MSTA.Following Error should not be set)')
        self.assertEqual(0, msta & self.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        bError   = self.pv_Err.get(use_monitor=False)
        nErrorId = self.pv_nErrorId.get(use_monitor=False)
        print '%s Error bError=%d nErrorId=%d' % (tc_no, bError, nErrorId)

        self.assertNotEqual(0, bError,   'bError')
        self.assertNotEqual(0, nErrorId, 'nErrorId')
        
        self.pv_nErrRst.put(1)

        msta = int(self.pv_MSTA.get(use_monitor=False))
        bError   = self.pv_Err.get(use_monitor=False)
        nErrorId = self.pv_nErrorId.get(use_monitor=False)
        print '%s Clean MSTA_BIT_PROBLEM=%x msta=%x bError=%d nErrorId=%d' % (tc_no, self.MSTA_BIT_PROBLEM, msta, bError, nErrorId)

        counter = 7
        while (msta & self.MSTA_BIT_MOVING or bError != 0 or nErrorId != 0):
            time.sleep(polltime)
            print '%s sleep counter = %d' % (tc_no, counter)
            msta = int(self.pv_MSTA.get(use_monitor=False))
            bError   = self.pv_Err.get(use_monitor=False)
            nErrorId = self.pv_nErrorId.get(use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break
        
        self.assertEqual(0, msta & self.MSTA_BIT_MOVING,  'Clean MSTA.Moving)')
        self.assertEqual(0, bError,   'bError')
        self.assertEqual(0, nErrorId, 'nErrorId')
        
        
