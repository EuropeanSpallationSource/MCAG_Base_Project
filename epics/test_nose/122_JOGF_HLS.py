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

MSTA_BIT_HOMED        =  1 << (15 -1)    #4000
MSTA_BIT_MINUS_LS     =  1 << (14 -1)    #2000
MSTA_BIT_COMM_ERR     =  1 << (13 -1)    #1000
MSTA_BIT_GAIN_SUPPORT =  1 << (12 -1)    #0800
MSTA_BIT_MOVING       =  1 << (11 -1)    #0400
MSTA_BIT_PROBLEM      =  1 << (10 -1)    #0200
MSTA_BIT_PRESENT      =  1 << (9 -1)     #0100
MSTA_BIT_HOME         =  1 << (8 -1)     #0080
MSTA_BIT_SLIP_STALL   =  1 << (7 -1)     #0040
MSTA_BIT_AMPON        =  1 << (6 -1)     #0020
MSTA_BIT_UNUSED       =  1 << (5 -1)     #0010
MSTA_BIT_HOMELS       =  1 << (4 -1)     #0008
MSTA_BIT_PLUS_LS      =  1 << (3 -1)     #0004
MSTA_BIT_DONE         =  1 << (2 -1)     #0002
MSTA_BIT_DIRECTION    =  1 << (1 -1)     #0001

def getMSTAtext(msta):
    ret = ''
    if (msta & MSTA_BIT_HOMED):
        ret = ret + 'Hd'
    else:
        ret = ret +'..'
    if (msta & MSTA_BIT_MINUS_LS):
        ret = ret + 'LLS'
    else:
        ret = ret +'...'
    #if (msta & MSTA_BIT_GAIN_SUPPORT):
    #    ret = ret + 'G'
    #else:
    #    ret = ret +'.'
    if (msta & MSTA_BIT_MOVING):
        ret = ret + 'Mov'
    else:
        ret = ret +'...'
    if (msta & MSTA_BIT_PROBLEM):
        ret = ret + 'P'
    else:
        ret = ret +'.'
    if (msta & MSTA_BIT_PRESENT):
        ret = ret + 'Enc'
    else:
        ret = ret +'...'
    if (msta & MSTA_BIT_HOME):
        ret = ret + 'Ho'
    else:
        ret = ret +'..'
    if (msta & MSTA_BIT_SLIP_STALL    ):
        ret = ret + 'Slip'
    else:
        ret = ret +'....'
    if (msta & MSTA_BIT_AMPON):
        ret = ret + 'Amp'
    else:
        ret = ret +'...'
    if (msta & MSTA_BIT_HOMELS):
        ret = ret + 'Hsw'
    else:
        ret = ret +'...'
    if (msta & MSTA_BIT_PLUS_LS):
        ret = ret + 'HLS'
    else:
        ret = ret +'...'
    if (msta & MSTA_BIT_DONE):
        ret = ret + 'Don'
    else:
        ret = ret +'...'
    return ret
        
    
def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def moveUserPosition(motor, tc_no, destination, velocity, acceleration):
    time_to_wait = 30
    if velocity > 0:
        distance = math.fabs(epics.caget(motor + '.RBV') - destination)
        time_to_wait += distance / velocity + 2 * acceleration
    print '%s: moveUserPosition Start destination=%f' % (tc_no, destination) 
    epics.caput(motor + '.VAL', destination, wait=True)
    msta = int(epics.caget(motor + '.MSTA'))
    rbv =  epics.caget(motor + '.RBV')
    print '%s: moveUserPosition End msta=%x %s rbv=%f' % (tc_no, msta, getMSTAtext(msta), rbv)



class Test(unittest.TestCase):

    m1 = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(m1 + '.HLM')
    llm = epics.caget(m1 + '.LLM')

    per70_UserPosition  = round((3 * llm + 7 * hlm) / 10)

    range_postion    = hlm - llm
    jogging_velocity = epics.caget(m1 + '.JVEL')
    moving_velocity  = epics.caget(m1 + '.VELO')
    acceleration     = epics.caget(m1 + '.ACCL')
    msta             = int(epics.caget(m1 + '.MSTA'))

    print 'llm=%f hlm=%f per70_UserPosition=%f' % (llm, hlm, per70_UserPosition)

    def setUp(self):
        print 'set up'

    def tearDown(self):
        print 'clean up'
        epics.caput(self.m1 + '.STOP', 1)

    # Assert if motor is homed
    def test_TC_1221(self):
        tc_no = "TC-1221"
        self.assertNotEqual(0, self.msta & MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        self.assertNotEqual(self.llm, self.hlm, 'llm must be != hlm')



    # high limit switch
    def test_TC_1222(self):
        if (self.msta & MSTA_BIT_HOMED):
            tc_no = "TC-1222-high-limit-switch"
            print '%s' % tc_no
            old_high_limit = epics.caget(self.m1 + '.HLM')
            old_low_limit = epics.caget(self.m1 + '.LLM')
            epics.caput(self.m1 + '.STOP', 1)
            #Go away from limit switch
            moveUserPosition(self.m1, tc_no, self.per70_UserPosition, self.moving_velocity, self.acceleration)
            #switch off the soft limits. Depending on the postion
            # low or high must be set to 0 first
            if epics.caget(self.m1 + '.RBV') > 0:
                epics.caput(self.m1 + '.LLM', 0.0)
                epics.caput(self.m1 + '.HLM', 0.0)
            else:
                epics.caput(self.m1 + '.HLM', 0.0)
                epics.caput(self.m1 + '.LLM', 0.0)

            epics.caput(self.m1 + '.JOGF', 1, wait=True)
            # Get values, check them later
            lvio = int(epics.caget(self.m1 + '.LVIO'))
            mstaE = int(epics.caget(self.m1 + '.MSTA'))
            #Go away from limit switch
            moveUserPosition(self.m1, tc_no, old_high_limit, self.moving_velocity, self.acceleration)
            print '%s msta=%x lvio=%d' % (tc_no, mstaE, lvio)

            epics.caput(self.m1 + '.LLM', old_low_limit)
            epics.caput(self.m1 + '.HLM', old_high_limit)

            #self.assertEqual(0, lvio, 'LVIO == 0')
            self.assertEqual(0, mstaE & MSTA_BIT_PROBLEM,    'No Error MSTA.Problem at PLUS_LS')
            self.assertEqual(0, mstaE & MSTA_BIT_MINUS_LS,   'Minus hard limit switch not active')
            self.assertNotEqual(0, mstaE & MSTA_BIT_PLUS_LS, 'Plus hard limit switch active')


