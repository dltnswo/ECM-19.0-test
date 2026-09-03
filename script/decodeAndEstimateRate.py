#!/usr/bin/python

import sys, os, subprocess

if len( sys.argv ) < 3:
    print( "Usage: decodeAndEstimateRate_parallel.py exePath bitstreamFile outputDirectory" )
    sys.exit()

if not os.path.isfile( sys.argv[2] ):
    print( "Error: Could not open bitstreamFile: '" + sys.argv[2] + "'" )
    sys.exit(-1)

workDir = os.path.abspath( os.getcwd() )
outDir = sys.argv[3]
binDir = sys.argv[1]

if not os.path.isdir( outDir ):
    print( "Error: outputDirectory doesn't exist." )
    sys.exit(-1)

xmlFile = "CabacBits_data.xml"
binFile = "CabacBits_data.bin"

if os.path.isfile( os.path.join(binDir, xmlFile) ):
    print( "Error: CabacBits_data.xml already exists." )
    sys.exit(-1)

if os.path.isfile( os.path.join(binDir, binFile) ):
    print( "Error: CabacBits_data.bin already exists." )
    sys.exit(-1)

logDir = sys.argv[2].strip(".bin") + ".log"
f = open(logDir, "w") 

subprocess.call( [ os.path.join( binDir, 'DecoderAppStatic' ), '-b', sys.argv[2] ], cwd=binDir, stdout=f )

command = os.path.join( binDir, 'RateEstimatorStatic' )
command += " " + os.path.join( binDir, xmlFile) + " " + os.path.join( binDir, binFile) + " " + outDir
os.system(command)

os.remove( os.path.join(binDir, xmlFile) )
os.remove( os.path.join(binDir, binFile) )




