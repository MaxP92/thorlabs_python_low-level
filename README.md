# thorlabs_python_low-level
Python code implementing Thorlabs APT communication protocol, by-passing APT.dll

Note : for APT.dll wrappers, see :
thorlabs_apt : https://github.com/qpit/thorlabs_apt
pyAPT : https://github.com/mcleung/PyAPT/

For another wrapper using low-level, see :
thorpy : https://github.com/UniNE-CHYN/thorpy

Motivations : 'LLSetGetDigOPs' method is not in APT.dll
So trigger IN/OUT cannot be set
also, APT.dll can be used by many thorlabs instrument at the same  time, leading to possible instabilities

I strongly recommend the reading of APT_Communications_Protocol

---------------------------------------------------------

Config used for testing : 
Windows
Python 3.5 (Anaconda 3)
Tried with a BBD102 controller for a MLS203 motor (Thorlabs) only
BBD102 Firmware version : 1.1.6 (1.1.1 does not have all the functions)

(other motors should work, code easily modifiable for other controllers)


Required libraries :
- pySerial : https://pythonhosted.org/pyserial/
OR 
- pyftdi : http://eblot.github.io/pyftdi/
OR 
- pylibftdi : https://pylibftdi.readthedocs.io/en/0.15.0/


Consider saying thanks ! --> maxime.pinsard@outlook.com


Copyright to the code is held by the following parties:
Copyright (C) 2017 Maxime Pinsard, INRS-EMT Varennes

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, version 2.1.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

Name: thorlabs_lowlevel
Version: 0.1
Summary: python routine for Thorlabs' lowlevel intrument control
Author: Maxime PINSARD
Author-email: maxime.pinsard@outlook.com

Platform: Windows
Classifier: Intended Audience :: Science/Research
Classifier: License :: OSI Approved :: GNU General Public License v2 (GPLv2)
Classifier: Programming Language :: Python
Classifier: Operating System :: Microsoft :: Windows
