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
        print '%s: wait_for_start=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_start, dmov, movn, epics.caget(motor + '.DRBV', use_monitor=False))
        if movn and not dmov:
           break
        time.sleep(polltime)

    wait_for_done = math.fabs(wait_for_done) #negative becomes positive
    wait_for_done += 1 # One extra second for rounding
    while wait_for_done > 0:
        dmov = int(epics.caget(motor + '.DMOV'))
        movn = int(epics.caget(motor + '.MOVN'))
        print '%s: wait_for_done=%f dmov=%d movn=%d dpos=%f' % (tc_no, wait_for_done, dmov, movn, epics.caget(motor + '.DRBV', use_monitor=False))
        if dmov and not movn:
            return True
        time.sleep(polltime)
        wait_for_done -= polltime
    return False

class Test(unittest.TestCase):
    MSTA_BIT_HOMED    = 1 << (15 -1)
    MSTA_BIT_MINUS_LS = 1 << (14 -1)
    MSTA_BIT_PLUS_LS  = 1 << (3 -1)

    m1 = os.getenv("TESTEDMOTORAXIS")
    print "m1=%s" % (m1)

    hlm = float(epics.caget(m1 + '.HLM'))
    llm = float(epics.caget(m1 + '.LLM'))
    range_postion    = hlm - llm
    homing_velocity  = epics.caget(m1 + '.HVEL')
    acceleration     = epics.caget(m1 + '.ACCL')

    print "llm=%f hlm=%f" % (llm, hlm)

    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'

    # Home the motor
    def test_TC_100(self):
        tc_no = "TC-100"
        print '%s Home the motor' % tc_no
	msta = int(epics.caget(self.m1 + '.MSTA'))
        if (msta & self.MSTA_BIT_PLUS_LS):
            epics.caput(self.m1 + '.HOMR', 1)
        else:
            epics.caput(self.m1 + '.HOMF', 1)
        time_to_wait = 30
        if self.range_postion > 0 and self.homing_velocity > 0:
            time_to_wait = 1 + self.range_postion / self.homing_velocity + 2 * self.acceleration

        # Homing velocity not implemented, wait longer
        time_to_wait = 180
        done = waitForStartAndDone(self.m1, tc_no, time_to_wait)

        msta = int(epics.caget(self.m1 + '.MSTA'))
        self.assertEqual(True, done, 'done = True')
        self.assertNotEqual(0, msta & self.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')



