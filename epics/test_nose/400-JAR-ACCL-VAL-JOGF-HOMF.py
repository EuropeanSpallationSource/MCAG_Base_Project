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
        dmov = int(epics.caget(motor + '.DMOV'))
        movn = int(epics.caget(motor + '.MOVN'))
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
        dmov = int(epics.caget(motor + '.DMOV'))
        movn = int(epics.caget(motor + '.MOVN'))
        rbv = epics.caget(motor + '.RBV')
        print '%s: wait_for_stop=%f dmov=%d movn=%d rbv=%f' % (tc_no, wait_for_stop, dmov, movn, rbv)
        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False

def getAcceleration(tc_no):
    print '%s: getAcceleration' % (tc_no)
    # TODO: MC_CPU1 needs to be a parameter
    epics.caput(os.getenv("TESTEDMCUASYN") + ".PORT", "MC_CPU1")

    epics.caput(os.getenv("TESTEDMCUASYN") + ".AOUT", "Main.M" + os.getenv("TESTEDMOTORADDR") + ".fAcceleration?")
    res = epics.caget (os.getenv("TESTEDMCUASYN") + ".AINP", as_string=True)
    print '%s: getAcceleration res=(%s)' % (tc_no, res)
    if res == "":
        time.sleep(polltime)
        res = epics.caget (os.getenv("TESTEDMCUASYN") + ".AINP", as_string=True)
        print '%s: getAcceleration res=(%s)' % (tc_no, res)
    return float(res + "0")

class Test(unittest.TestCase):
    MSTA_BIT_MOVING   = 1 << (11 -1)

    m1 = os.getenv("TESTEDMOTORAXIS")
    pv_MSTA = epics.PV(os.getenv("TESTEDMOTORAXIS") + ".MSTA")
    pv_MCU_AOUT = epics.PV(os.getenv("TESTEDMCUASYN") + ".AOUT")
    pv_MCU_AINP = epics.PV(os.getenv("TESTEDMCUASYN") + ".AINP")

    saved_HLM = epics.caget(m1 + '.HLM')
    saved_LLM = epics.caget(m1 + '.LLM')
    saved_CNEN = epics.caget(m1 + '.CNEN')

    def setUp(self):
        print 'set up'


    def tearDown(self):
        print 'clean up'


    # 10% UserPosition
    def test_TC_401(self):
        tc_no = "TC-401-10-percent-dialPosition"
        print '%s' % tc_no
        print '%s' % tc_no
        epics.caput(self.m1 + '.CNEN', 1)
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        epics.caput(self.m1 + '.VAL', destination, wait=True)

    # 20% UserPosition, check acceleration
    def test_TC_402(self):
        tc_no = "TC-402-20-percent-dialPosition"
        print '%s' % tc_no
        saved_ACCL = epics.caget(self.m1 + '.ACCL')

        used_ACCL = saved_ACCL + 1.0 # Make sure we have an acceleration != 0
        epics.caput(self.m1 + '.ACCL', used_ACCL)

        destination =  (2 * self.saved_HLM + 8 * self.saved_LLM) / 10
        epics.caput(self.m1 + '.VAL', destination, wait=True)
        rbv = epics.caget(self.m1 + '.RBV')
        epics.caput(self.m1 + '.ACCL', saved_ACCL)
        saved_ACCL = None

        saved_VELO = epics.caget(self.m1 + '.VELO')
        expacc = saved_VELO / used_ACCL
        resacc = float(getAcceleration(tc_no))
        print '%s %s ACCL=%f VELO=%f expacc=%f resacc=%f' % (tc_no,self.m1, used_ACCL,saved_VELO,expacc,resacc)
        assert calcAlmostEqual(self.m1, tc_no, expacc, resacc, 2)
        assert calcAlmostEqual(self.m1, tc_no, destination, rbv, 2)


    # Jog, wait for start, stop. check fAcceleration
    def test_TC_405(self):
        tc_no = "TC-405-JOG-fAcceleration"
        print '%s' % tc_no
        saved_JAR = epics.caget(self.m1 + '.JAR')
        used_JAR = saved_JAR + 0.5 # Make sure we have an acceleration != 0
        epics.caput(self.m1 + '.JAR', used_JAR)

        epics.caput(self.m1 + '.JOGF', 1)
        ret = waitForStart(self.m1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStart return True')

        epics.caput(self.m1 + '.JOGF', 0)
        ret = waitForStop(self.m1, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStop return True')
        epics.caput(self.m1 + '.JAR', saved_JAR)
        saved_JAR = None

        expacc = used_JAR
        resacc = float(getAcceleration(tc_no))
        print '%s expacc=%f resacc=%f' % (tc_no,expacc,resacc)
        assert calcAlmostEqual(self.m1, tc_no, expacc, resacc, 2)

