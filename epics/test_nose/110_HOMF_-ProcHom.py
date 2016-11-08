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

def jogDirection(motor, tc_no, direction, time_to_wait):
    if direction > 0:
        epics.caput(motor + '.JOGF', 1)
    else:
        epics.caput(motor + '.JOGR', 1)

    done = waitForStartAndDone(motor, tc_no + " jogDirection", 30 + time_to_wait + 3.0)

    if direction > 0:
        epics.caput(motor + '.JOGF', 0)
    else:
        epics.caput(motor + '.JOGR', 0)

def movePosition(motor, tc_no, destination, velocity, acceleration):
    time_to_wait = 30
    if velocity > 0:
        distance = math.fabs(epics.caget(motor + '.RBV') - destination)
        time_to_wait += distance / velocity + 2 * acceleration
    epics.caput(motor + '.VAL', destination)
    done = waitForStartAndDone(motor, tc_no + " movePosition", time_to_wait)

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def homeTheMotor(tself, motor, tc_no, procHome, jogToLSBefore):
    old_high_limit = epics.caget(motor + '.HLM')
    old_low_limit = epics.caget(motor + '.LLM')
    old_ProcHom   = tself.pv_ProcHom.get(use_monitor=False)
    if jogToLSBefore != 0:
        if motor.get_position(readback=True) > 0:
            epics.caput(motor + '.LLM', 0.0)
            epics.caput(motor + '.HLM', 0.0)
        else:
            epics.caput(motor + '.HLM', 0.0)
            epics.caput(motor + '.LLM', 0.0)
        # soft limit range assumed to be = hard range /1.5
        time_to_wait = 1.5 * (old_high_limit - old_low_limit) / motor.JVEL + 2 * motor.ACCL

        jogDirection(motor, tc_no, jogToLSBefore, time_to_wait)
        epics.caput(motor + '.LLM', old_low_limit)
        epics.caput(motor + '.HLM', old_high_limit)
    else:
        movePosition(motor, tc_no, (old_high_limit + old_low_limit) / 2.0,
                     tself.moving_velocity, tself.acceleration)

    tself.pv_ProcHom.put(procHome)
    msta = int(epics.caget(motor + '.MSTA'))
    if (msta & tself.MSTA_BIT_PLUS_LS):
        epics.caput(motor + '.HOMR', 1)
    else:
        epics.caput(motor + '.HOMF', 1)

    time_to_wait = 180
    done = waitForStartAndDone(motor, tc_no + " homeTheMotor", time_to_wait)

    msta = int(epics.caget(motor + '.MSTA'))
    homed = 0
    if (msta & tself.MSTA_BIT_HOMED):
        homed = 1
    print '%s: homeTheMotor msta=%x homed=%d' % (tc_no, msta, homed)
    tself.assertEqual(True, done,                          tc_no +  "done = True")
    tself.assertEqual(0, msta & tself.MSTA_BIT_SLIP_STALL, tc_no + "MSTA.no MSTA_BIT_SLIP_STALL")
    tself.assertNotEqual(0, homed,   tc_no + "MSTA.homed (Axis has been homed)")
    tself.pv_ProcHom.put(old_ProcHom, wait=True)

class Test(unittest.TestCase):
    MSTA_BIT_HOMED      = 1 << (15 -1)
    MSTA_BIT_MINUS_LS   = 1 << (14 -1)
    MSTA_BIT_PROBLEM    = 1 << (10 -1)
    MSTA_BIT_SLIP_STALL = 1 << (7 -1)
    MSTA_BIT_PLUS_LS    = 1 << (3 -1)
    m1 = os.getenv("TESTEDMOTORAXIS")
    pv_ProcHom = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ProcHom")


    hlm = float(epics.caget(m1 + '.HLM'))
    llm = float(epics.caget(m1 + '.LLM'))
    range_postion    = hlm - llm
    homing_velocity  = epics.caget(m1 + '.HVEL')
    acceleration     = epics.caget(m1 + '.ACCL')
    moving_velocity  = epics.caget(m1 + '.VELO')
    acceleration     = epics.caget(m1 + '.ACCL')

    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'

    # Home against low limit switch
    #def test_TC_11110(self):
    #    tc_no = "TC-11110"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 1, 1)

    def test_TC_11111(self):
        tc_no = "TC-11111"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 1, 0)

    #def test_TC_11112(self):
    #    tc_no = "TC-11112"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 1, -1)

    # Home against high limit switch
    #def test_TC_11120(self):
    #    tc_no = "TC-11120"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 2, 1)

    def test_TC_11121(self):
        tc_no = "TC-11121"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 2, 0)

    #def test_TC_11122(self):
    #    tc_no = "TC-11122"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 2, -1)

    # Home against home switch via LLS
    #def test_TC_11130(self):
    #    tc_no = "TC-11130"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 3, 1)

    def test_TC_11131(self):
        tc_no = "TC-11131"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 3, 0)

    #def test_TC_11132(self):
    #    tc_no = "TC-11132"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 3, -1)

    # Home against home switch, via HLS
    #def test_TC_11140(self):
    #    tc_no = "TC-11140"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 4, 1)

    def test_TC_11141(self):
        tc_no = "TC-11141"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 4, 0)

    #def test_TC_11142(self):
    #    tc_no = "TC-11142"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 4, -1)

    # Home against home switch via LLS
    #def test_TC_11150(self):
    #    tc_no = "TC-11150"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 5, 1)

    def test_TC_11151(self):
        tc_no = "TC-11151"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 5, 0)

    #def test_TC_11152(self):
    #    tc_no = "TC-11152"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 5, -1)

    # Home against home switch, via HLS
    #def test_TC_11160(self):
    #    tc_no = "TC-11160"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 6, 1)

    def test_TC_11161(self):
        tc_no = "TC-11161"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, 6, 0)

    #def test_TC_11162(self):
    #    tc_no = "TC-11162"
    #    print '%s Home ' % tc_no
    #    homeTheMotor(self, self.m1, tc_no, 6, -1)

    # Need to home with the original homing procedure
    def test_TC_11191(self):
        old_ProcHom   = self.pv_ProcHom.get(use_monitor=False)
        tc_no = "TC-11191"
        print '%s Home ' % tc_no
        homeTheMotor(self, self.m1, tc_no, old_ProcHom, 0)

