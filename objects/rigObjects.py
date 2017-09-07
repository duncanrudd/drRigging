import drRigging.python.utils.coreUtils as coreUtils
import pymel.core as pmc
reload(coreUtils)


def nonTwistingChild(node, name, axis='x'):
    '''
    calls isolateTwist function supplying node's world and inverse parent matrices
    creates a new group parented under node which negates the twisting axis
    :param node: node whose twist you wish to extract
    :param name: prefix for newly created nodes
    :param axis: axis to negate
    :return: [multMatrix, decomposeMatrix, eulerToQuat, newChildGroup]
    '''
    twist = coreUtils.isolateTwist(node.worldMatrix[0], node.parentInverseMatrix[0], name, axis=axis)
    g = coreUtils.addChild(node, 'group', name='%s_noTwist_srt' % name)
    twist.append(g)
    twistSourceAttr = pmc.Attribute('%s.outputRotate%s' % (twist[2].name(), axis[-1].upper()))
    twistDestAttr = pmc.Attribute('%s.r%s' % (g.name(), axis[-1]))
    c = coreUtils.convert(twistSourceAttr, -1, name='%s_twistInvert_utl' % name)
    c.output.connect(twistDestAttr)
    return g

def orientationAverageNode(start, end, name, endAsUp=1, axis='x', upAxis='y'):
    '''
    creates a transform which averages the orientation between start and end on a 2D plane.
    Useful for adding a bridging joint in knees, elbows, wrists etc.
    :param start: first node whose rotation to consider
    :param end: second node whose rotation to consider
    :param name: prefix for newly created nodes
    :param endAsUp: if true, end will be used when computing up vectors. Otherwise start will be used.
    :param axis: direction of aim vectors
    :param upAxis: axis to use when computing up vectors
    :return: transform representing the averaged orientation
    '''
    g = pmc.group(empty=1, name='%s_srt' % name)

    startAimVec = coreUtils.matrixAxisToVector(start, '%s_startAimVec_utl' % name, axis=axis)
    endAimVec = coreUtils.matrixAxisToVector(end, '%s_endAimVec_utl' % name, axis=axis)
    avgAimVec = coreUtils.average([endAimVec.output, startAimVec.output], '%s_avgAimVec_utl' % name)
    avgAimVecNormalized = coreUtils.normalizeVector(avgAimVec.output3D, '%s_avgAimVecNormalized_utl' % name)

    upNode = end
    if not endAsUp:
        upNode = start

    upVec = coreUtils.matrixAxisToVector(upNode, '%s_upVec_utl' % name, axis=upAxis)
    sideVec = coreUtils.cross(avgAimVecNormalized.output, upVec.output, '%s_sideVec_utl' % name)
    d = coreUtils.isDecomposed(end)
    d.outputScale.connect(g.s)

    avgMatx = pmc.createNode('fourByFourMatrix', name='%s_avgMtx_utl' % name)

    orderDict = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}
    matrixList = ['', '', '']
    matrixList[orderDict[axis]] = 'X'
    matrixList[orderDict[upAxis]] = 'Y'

    avgAimVecNormalized.outputX.connect(pmc.Attribute('%s.in%s0' % (avgMatx.name(), matrixList.index('X'))))
    avgAimVecNormalized.outputY.connect(pmc.Attribute('%s.in%s1' % (avgMatx.name(), matrixList.index('X'))))
    avgAimVecNormalized.outputZ.connect(pmc.Attribute('%s.in%s2' % (avgMatx.name(), matrixList.index('X'))))

    upVec.outputX.connect(pmc.Attribute('%s.in%s0' % (avgMatx.name(), matrixList.index('Y'))))
    upVec.outputY.connect(pmc.Attribute('%s.in%s1' % (avgMatx.name(), matrixList.index('Y'))))
    upVec.outputZ.connect(pmc.Attribute('%s.in%s2' % (avgMatx.name(), matrixList.index('Y'))))

    sideVec.outputX.connect(pmc.Attribute('%s.in%s0' % (avgMatx.name(), matrixList.index(''))))
    sideVec.outputY.connect(pmc.Attribute('%s.in%s1' % (avgMatx.name(), matrixList.index(''))))
    sideVec.outputZ.connect(pmc.Attribute('%s.in%s2' % (avgMatx.name(), matrixList.index(''))))

    d.outputTranslateX.connect(avgMatx.in30)
    d.outputTranslateY.connect(avgMatx.in31)
    d.outputTranslateZ.connect(avgMatx.in32)

    d = coreUtils.decomposeMatrix(avgMatx.output, name='%s_avgMtxToSrt_utl' % name)
    d.outputTranslate.connect(g.t)
    d.outputRotate.connect(g.r)

    return g
