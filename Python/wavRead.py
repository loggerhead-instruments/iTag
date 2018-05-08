#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 15 18:16:32 2018

@author: dmann
"""
import matplotlib.pyplot as plt
import scipy.io.wavfile as wav

path = '/Users/dmann/w/iTag/example/'

fileName = '01002743.AMX_3D_HMS_ 0_27_43__DMY_ 1_ 1_ 0.wav'
s, y = wav.read(path + fileName)