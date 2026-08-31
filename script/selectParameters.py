#!/usr/bin/python

from __future__ import division
from __future__ import print_function

import sys, os
#from __builtin__ import False

BITS_SHIFT = 30

shiftIdxList = [ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13 ]
rateIdxList = [ 4, 11, 18, 25, 32]

if len( sys.argv ) != 2:
    print( "Usage: selectParameters.py ratesDirectory" )
    sys.exit()

if not os.path.isdir( sys.argv[1] ):
    print( "Error: Could not access ratesDirectory: '" + sys.argv[2] + "'" )
    sys.exit(-1)



def checkSeqFoldersAndGetCtxList( sfPath, sf ):
    if len( sf ) == 0:
        print( "Error: Could not find any seqence folders." )
        sys.exit(-1)

    print( "Checking consistency of 'CabacBits_data_index.txt' files in %d folders..." % len( sf ) )

    firstSeqLines = None
    seqBits = []
    for s in sf:
        idxFile = os.path.join( sfPath, s, "CabacBits_data_index.txt" )
        if not os.path.isfile( idxFile ):
            print( "Error: Could not access index file: '" + idxFile + "'" )
            sys.exit(-1)
        with open( idxFile, "rt" ) as f:
            sfLines = [ x.rstrip() for x in f.readlines() ]
            seqBits.append( int( sfLines[0] ) )
            sfLines = sfLines[1:]
        if firstSeqLines is None:
            firstSeqLines = sfLines
        else:
            if sfLines != firstSeqLines:
                print( "Error: CabacBits_data_index.txt file in folder '%s' doesn't contain the same list of ctx files in the same order as in folder '%s'." % (s, sf[0]) )
                sys.exit(-1)
    print( "Success." )
    return firstSeqLines, seqBits


seqFolders = [x for x in os.listdir( sys.argv[1] ) if os.path.isdir( os.path.join( sys.argv[1], x ) ) ]

ctxList, seqBits = checkSeqFoldersAndGetCtxList( sys.argv[1], seqFolders )

def parseCtxFile( ctxFileName, numInits, sequenceBits, bits ):
    try:
        with open( ctxFileName, "rt" ) as f:
            ctx = [ x.rstrip() for x in f.readlines() ]
    except:
        print( "Could not access ctx file: %s", f )
        sys.exit(-1)
    runIdx = 0
    initBitsList = False
    if len( bits ) == 0:
        initBitsList = True
    for initType in range( 3 ):
        if initBitsList:
            bits.append([])
        for ws in range( 13 ):
            if initBitsList:
                bits[initType].append([])
            for adaRate in range (5):            
                if initBitsList:
                    bits[initType][ws].append([])
            
                initBits = [x for x in ctx[runIdx].split( ',' ) ]
                if initBits[-1].strip() == '':
                    initBits = initBits[:-1]
                initBits = [int(x) for x in initBits ]
            
                if numInits is None:
                    numInits = len( initBits )
                    print( "Found %d init value candidates." % numInits )
                else:
                    if numInits != len( initBits ):
                        print( "Error: File '%s' seems inconsistent." % ctxFileName )
                        sys.exit(-1)
        
                if initBitsList:
                    bits[initType][ws][adaRate] = [(x << (BITS_SHIFT-15)) // sequenceBits for x in initBits]
                else:
                    bits[initType][ws][adaRate] = [bits[initType][ws][adaRate][i] + (x << (BITS_SHIFT-15)) // sequenceBits for i, x in enumerate(initBits)]
                runIdx += 1

        if initType < 2 and ctx[runIdx] != "":
            print( "Unexpected nonempty line between two blocks of bit rates." )
            sys.exit(-1)
        else:
            runIdx += 1
    offset = [x for x in ctx[runIdx].split( ',' ) ]            
    offset0 = int(offset[0])
    offset1 = int(offset[1])
    initB   = int(offset[2])
    rateB   = int(offset[3])
    weightB = int(offset[4])
    initP   = int(offset[5])
    rateP   = int(offset[6])
    weightP = int(offset[7])
    initI   = int(offset[8])
    rateI   = int(offset[9])
    weightI = int(offset[10])
    return numInits, bits, offset0, offset1, initB, rateB, weightB, initP, rateP, weightP, initI, rateI, weightI


lastCtxName = None
ctxArray = []
allCtx = []

numInits = None
for ctxFile in ctxList:
    ctxBits = []
    #input()
    for i, seq in enumerate( seqFolders ):
        ctxFn = os.path.join( sys.argv[1], seq, ctxFile )
        numInits, ctxBits, offset0, offset1, initB, rateB, weightB, initP, rateP, weightP, initI, rateI, weightI = parseCtxFile( ctxFn, numInits, seqBits[i], ctxBits )
    print( "Processing %s" % ctxFile )
    #print("ctxBits ", ctxBits)
    
    # create new array
    _, _, ctxName, _ = ctxFile.split( "_" )
    if ctxName != lastCtxName and lastCtxName is not None:
        allCtx.append( [ lastCtxName, zip( *ctxArray ) ] )
        ctxArray = []
    
    lastCtxName = ctxName
    
    bestBits = 0
    bestLine = []
    minInit = []
    minInitIdx = []
    for sliceTypeIdx, sliceType in enumerate(ctxBits):
        minInit.append([])
        minInitIdx.append([])
        for wsIdx, ws in enumerate(sliceType):
            minInit[sliceTypeIdx].append([])
            minInitIdx[sliceTypeIdx].append([])
            for adaRateIdx, adaRate in enumerate(ws):
                minInit[sliceTypeIdx][wsIdx].append(min(adaRate))
                minInitIdx[sliceTypeIdx][wsIdx].append( adaRate.index( min(adaRate) ) )
        #print("\n")
        #print("minInit", minInit)
        #print("\n")
        #print("minInitIdx", minInitIdx)
        
        # decide adaptive rate for each window size
        minAdaRate = []
        minAdaRateIdx = []
        for sliceTypeIdx, sliceType in enumerate( minInit ):
            minAdaRate.append([])
            minAdaRateIdx.append([])
            for wsIdx, ws in enumerate(sliceType):
                minAdaRate[sliceTypeIdx].append( min(ws) )
                minAdaRateIdx[sliceTypeIdx].append( ws.index( min(ws) ) )
        #print("\n")
        #print("minAdaRate", minAdaRate)
        #print("\n")
        #print("minAdaRateIdx", minAdaRateIdx)
        
        # decide window size for each sliceType
        currLine = []
        totalBits = 0
        for sliceTypeIdx, sliceType in enumerate( minAdaRate ):
            f_minWs = min( sliceType )
            f_minWsIdx = sliceType.index( f_minWs )
            f_minAdaRateIdx = minAdaRateIdx[sliceTypeIdx][f_minWsIdx]
            f_minInitIdx = minInitIdx[sliceTypeIdx][f_minWsIdx][f_minAdaRateIdx]
            #print("f_minWs", f_minWs)
            #print("f_minWsIdx", f_minWsIdx)
            #print("f_minAdaRateIdx", f_minAdaRateIdx)
            #print("f_minInitIdx", f_minInitIdx)
            totalBits += f_minWs
            if sliceTypeIdx == 0:
                if f_minWs <= 0:
                    currLine.append(-1)
                    currLine.append(-1)
                    currLine.append(-1)
                else:
                    currLine.append( f_minInitIdx )
                    currLine.append( shiftIdxList[f_minWsIdx] )
                    currLine.append( rateIdxList[f_minAdaRateIdx] )
            elif sliceTypeIdx == 1:
                currLine.append(initP)
                currLine.append(rateP)
                currLine.append(weightP)
            elif sliceTypeIdx == 2:
                currLine.append(initB)
                currLine.append(rateB)
                currLine.append(weightB)
        currLine.append( offset0 )
        currLine.append( offset1 )
    
        #print("\n")
        #print("currLine", currLine)
        #print("totalBits", totalBits)
    #print("\n")
    #print("bestLine", bestLine)
    #print("bestBits", bestBits)
    ctxArray.append( currLine )
        
    if ctxFile == ctxList[-1]:
        allCtx.append( [ lastCtxName, zip( *ctxArray ) ] )


indent = ""
for name, [ I, WS_I, ADA_I, P, WS_P, ADA_P, B, WS_B, ADA_B, OFF0, OFF1 ] in allCtx:
    I  = "".join( [ "%3d, " % x if x >= 0 else "CNU, " for x in I ] )
    P  = "".join( [ "%3d, " % x if x >= 0 else "CNU, " for x in P ] )
    B  = "".join( [ "%3d, " % x if x >= 0 else "CNU, " for x in B ] )
    WS_I = "".join( [ "%3d, " % x if x >= 0 else "DWS, " for x in WS_I ] )
    WS_P = "".join( [ "%3d, " % x if x >= 0 else "DWS, " for x in WS_P ] )
    WS_B = "".join( [ "%3d, " % x if x >= 0 else "DWS, " for x in WS_B ] )
    ADA_I = "".join( [ "%3d, " % x if x >= 0 else "DWE, " for x in ADA_I ] )
    ADA_P = "".join( [ "%3d, " % x if x >= 0 else "DWE, " for x in ADA_P ] )
    ADA_B = "".join( [ "%3d, " % x if x >= 0 else "DWE, " for x in ADA_B ] )
    OFF0 = "".join( [ "%3d, " % x if x >= 0 else "DWO, " for x in OFF0 ] )
    OFF1 = "".join( [ "%3d, " % x if x >= 0 else "DWO, " for x in OFF1 ] )
    nameParts = name.split( "[" )
    if len( nameParts ) > 1:
        if( nameParts[1] == "0]" ):
            if indent != "":
                print( "};\n")
            print( "const CtxSet ContextSetCfg::%s[] = " % nameParts[0] )
            print( "{" )
        indent = "  "
        print( "%sContextSetCfg::addCtxSet" % indent )
    else:
        if indent != "":
            print( "};\n")
        indent = ""
        print( "const CtxSet ContextSetCfg::%s = ContextSetCfg::addCtxSet" % name )
    print( "%s({" % indent );
    print( "%s  { %s}," % (indent, B ) )
    print( "%s  { %s}," % (indent, P ) )
    print( "%s  { %s}," % (indent, I ) )
    print( "%s  { %s}," % (indent, WS_B) )
    print( "%s  { %s}," % (indent, WS_P) )
    print( "%s  { %s}," % (indent, WS_I) )
    print( "%s  { %s}," % (indent, ADA_B) )
    print( "%s  { %s}," % (indent, ADA_P) )
    print( "%s  { %s}," % (indent, ADA_I) )
    print( "%s  { %s}," % (indent, OFF0) )
    print( "%s  { %s}," % (indent, OFF1) )
    #print( "%s  { %s}," % (indent, WS) )
    print( "%s})%s" % ( indent, ";\n" if indent == "" else "," ) )
if indent != "":
    print( "};\n")




