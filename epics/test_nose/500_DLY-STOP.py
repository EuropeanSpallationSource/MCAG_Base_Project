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

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = os.getenv("TESTEDMOTORAXIS")
    pvm1DLY  = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.DLY')
    pvm1ACCL = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.ACCL')
    pvm1VELO = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.VELO')
    pvm1DVAL = epics.PV(os.getenv("TESTEDMOTORAXIS") + '.DVAL')

    saved_HLM = epics.caget(m1 + '.HLM')
    saved_LLM = epics.caget(m1 + '.LLM')
    
    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'

    
    # 10% dialPosition
    def test_TC_501(self):
        tc_no = "TC_501-10-percent-dialPosition"
        print '%s' % tc_no
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        epics.caput(self.m1 + '.VAL', destination, wait=True)
        rbv = epics.caget(self.m1 + '.RBV')
        print '%s postion=%f' % (tc_no, rbv)
        assert calcAlmostEqual(self.m1, tc_no, destination, rbv, 2)
        
    # 10% dialPosition + X
    def test_TC_502(self):
        tc_no = "TC_502-10-percent-plus-1"
        print '%s' % tc_no
        rbv = epics.caget(self.m1 + '.RBV')
        saved_DLY  = epics.caget(self.m1 + '.DLY')
        saved_VELO = epics.caget(self.m1 + '.VELO')
        saved_ACCL = epics.caget(self.m1 + '.ACCL')

        epics.caput(self.m1 + '.DLY',5.2)
        epics.caput(self.m1 + '.VELO', 1)
        epics.caput(self.m1 + '.ACCL', 1)
        epics.caput(self.m1 + '.VAL', rbv + 1.0, wait=False)

        time.sleep(4.0)
        movn1 = epics.caget(self.m1 + '.MOVN')
        epics.caput(self.m1 + '.STOP', 1)
        time.sleep(7.0)
        dmov2 = epics.caget(self.m1 + '.DMOV')
        epics.caput(self.m1 + '.SPMG', 0)
        epics.caput(self.m1 + '.SPMG', 3)
        time.sleep(4.0)
        dmov3 = epics.caget(self.m1 + '.DMOV')
        print '%s: movn1=%d dmov2=%d dmov3=%d' % (tc_no, movn1, dmov2, dmov3)
        epics.caput(self.m1 + '.DLY',  saved_DLY)
        epics.caput(self.m1 + '.VELO', saved_VELO)
        epics.caput(self.m1 + '.ACCL', saved_ACCL)
        assert(dmov3)
        #assert(dmov2)
