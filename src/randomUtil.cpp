/* $Id: randomUtil.cpp,v 1.2 2009/03/04 18:02:40 welle Exp $ */
/********************************************************
 * Author: Jochen Welle
 *
 * Erstellt im Rahmen der Diplomarbeit:
 * "FastSLAM basierte Erstellung eines 3D-Umgebungsmodells
 *  während der Fahrt" von Jochen Welle
 *
 * Copyright (c) 2009 Jochen Welle. All rights reserved.
 ********************************************************/
#include <cstdlib>

#include "randomUtil.h"

// namespace drafting3d {

Random RandGlobal::randomInstance;
boost::mutex RandGlobal::randomMutex;


// } // end of namespace
