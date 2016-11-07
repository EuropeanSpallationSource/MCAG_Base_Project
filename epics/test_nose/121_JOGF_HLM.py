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

def waitForStartAndDone(motor, tc_no, wait_for_done):
    wait_for_start = 2
    while wait_for_start > 0:
        wait_for_start -= polltime
        dmov = int(epics.caget(motor + '.DMOV'))
        movn = int(epics.caget(motor + '.MOVN'))
        print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_start, dmov, movn, epics.caget(motor + '.RBV', use_monitor=False))
        if movn and not dmov:
           break
        time.sleep(polltime)

    wait_for_done = math.fabs(wait_for_done) #negative becomes positive
    wait_for_done += 1 # One extra second for rounding
    while wait_for_done > 0:
        dmov = int(epics.caget(motor + '.DMOV'))
        movn = int(epics.caget(motor + '.MOVN'))
        print '%s: wait_for_done=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_done, dmov, movn, epics.caget(motor + '.RBV', use_monitor=False))
        if dmov and not movn:
            return True
        time.sleep(polltime)
        wait_for_done -= polltime
    return False

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def moveUserPosition(motor, tc_no, destination, velocity, acceleration):
    time_to_wait = 30
    if velocity > 0:
        distance = math.fabs(epics.caget(motor + '.RBV', use_monitor=False) - destination)
        time_to_wait += distance / velocity + 2 * acceleration
    epics.caput(motor + '.VAL', destination)
    done = waitForStartAndDone(motor, tc_no, time_to_wait)


class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PROBLEM  = 1 << (10 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(m1 + '.HLM')
    llm = epics.caget(m1 + '.LLM')

    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    range_postion    = hlm - llm
    jogging_velocity = epics.caget(m1 + '.JVEL')
    moving_velocity  = epics.caget(m1 + '.VELO')
    acceleration     = epics.caget(m1 + '.ACCL')
    msta             = int(epics.caget(m1 + '.MSTA'))

    print 'llm=%f hlm=%f per90_UserPosition=%f' % (llm, hlm, per90_UserPosition)

    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'

    # Assert if motor is not homed
    def test_TC_1211(self):
        tc_no = "TC-1211"
        if not (self.msta & self.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')


    # per90 UserPosition
    def test_TC_1212(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1212-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            moveUserPosition(self.m1, tc_no, destination,
                             self.moving_velocity, self.acceleration)

            UserPosition = epics.caget(self.m1 + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            assert calcAlmostEqual(self.m1, tc_no, destination, UserPosition, 2)

    # High soft limit JOGF
    def test_TC_1213(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1213-low-soft-limit JOGF"
            print '%s' % tc_no
            epics.caput(self.m1 + '.JOGF', 1, wait=True)
            lvio = int(epics.caget(self.m1 + '.LVIO'))
            msta = int(epics.caget(self.m1 + '.MSTA'))

            self.assertEqual(0, msta & self.MSTA_BIT_PROBLEM,  'No MSTA.Problem JOGF')
            self.assertEqual(0, msta & self.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & self.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            epics.caput(self.m1 + '.JOGF', 0)


    # per90 UserPosition
    def test_TC_1214(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1214-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            moveUserPosition(self.m1, tc_no, destination,
                             self.moving_velocity, self.acceleration)

            UserPosition = epics.caget(self.m1 + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            assert calcAlmostEqual(self.m1, tc_no, destination, UserPosition, 2)

    # High soft limit JOGR + DIR
    def test_TC_1215(self):
        if (self.msta & self.MSTA_BIT_HOMED):
            tc_no = "TC-1215-low-soft-limit JOGF DIR"
            print '%s' % tc_no
            saved_DIR = epics.caget(self.m1 + '.DIR')
            saved_FOFF = epics.caget(self.m1 + '.FOFF')
            epics.caput(self.m1 + '.FOFF', 1)
            epics.caput(self.m1 + '.DIR', 1)
            epics.caput(self.m1 + '.JOGR', 1, wait=True)

            lvio = int(epics.caget(self.m1 + '.LVIO'))
            msta = int(epics.caget(self.m1 + '.MSTA'))

            self.assertEqual(0, msta & self.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & self.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & self.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF DIR')
            ### commit  4efe15e76cefdc060e14dbc3 needed self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            epics.caput(self.m1 + '.JOGF', 0)
            epics.caput(self.m1 + '.DIR', saved_DIR)
            epics.caput(self.m1 + '.FOFF', saved_FOFF)



