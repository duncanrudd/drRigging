import math

import maya.api._OpenMaya_py2 as om2
import maya.cmds as cmds
import pymel.core as pmc


def colorize( color=None, nodeList=[] ):
    '''
    takes a node ( or list or nodes ) and enables the drawing overrides.
    'Color' specifies either an integer for the required color or a string corresponding to a key in colorDict
    if nodelist is not supplied, will attempt to work on selected nodes.

    '''
    if not color:
        raise RuntimeError, 'color not specified. You must supply either a string or integer.'

    colorDict = {
                   'center':14, # green
                   'right':13, # red
                   'left':6, # blue
                   'red':13,
                   'blue':6,
                   'yellow':17,
                   'green':14,
                   'purple':9,
                   'cn':14, # green
                   'rt':13, # red
                   'lf':6, # blue
                  }

    if type(color) == type('hello') or type(color) == type(u'hello'):
        color = colorDict[color]

    if not nodeList:
        nodeList = pmc.selected()
    if type(nodeList) == type('hello') or type(nodeList) == type(u'hello'):
        nodeList = [pmc.PyNode(nodeList)]

    for n in nodeList:
        n.overrideEnabled.set(1)
        n.overrideColor.set(color)

def addChild(parent, childType, name, zero=1):
    '''
    adds a new node of type childType. Parents it to the parent node.
    :param childType: 'group', 'joint', 'locator'
    :return: newly created child node
    '''
    node = None
    if childType == 'group':
        node = pmc.group(empty=1, name=name)
        node.setParent(parent)
    elif childType == 'locator':
        node = pmc.spaceLocator(name=name)
        node.setParent(parent)
    elif childType == 'joint':
        node = pmc.joint(name=name)
        if not node.getParent() == parent:
            node.setParent(parent)
    if node:
        if zero:
            align(node, parent)
        return node
    else:
        return 'addChild: node not created'


def addParent(child, parentType, name, zero=1):
    '''
    adds a new node of type parentType. Parents node to it.
    :param childType: 'group', 'joint', 'locator'
    :return: newly created parent node
    '''
    node = None
    if not child:
        child = pmc.selected()[0]

    parent = pmc.listRelatives(child, p=1, fullPath=1)
    if type(parent) == type([]):
        if len(parent) > 0:
            parent = parent[0]

    if parentType == 'group':
        node = pmc.group(empty=1, name=name)
    elif parentType == 'locator':
        node = pmc.spaceLocator(name=name)

    if node:
        if zero:
            align(node, child)
        if parent:
            node.setParent(parent)
        child.setParent(node)
        return node
    else:
        return 'addParent: node not created'

def addNode(nodeType, name):
    '''
    creates a new node in the scene with the specified name
    :param nodeType: type of node to create. Options are 'group', 'joint', 'locator'
    :param name: Name of new node
    :return: newly created node
    '''
    node = None
    if nodeType == 'group':
        node = cmds.group(empty=1, name=name)
    elif nodeType == 'locator':
        node = cmds.spaceLocator(name=name)[0]
    elif nodeType == 'joint':
        sel = cmds.ls(sl=1)
        cmds.select([])
        node = cmds.joint()
        cmds.select(sel)

    if node:
        return node
    else:
        return 'addNode: node not created'

def align( node=None, target=None, translate=True, orient=True, scale=False, parent=0 ):
    '''
    sets the translation and / or orientation of node to match target
    If optional parent flag is set to true. Will also parent to the target node
    '''

    # Validate that the correct arguments have been supplied
    if not node or not target:
        # If node and target aren't explicitly supplied, check for a valid selection to use
        sel = pmc.selected()
        if len(sel) == 2:
            node, target = sel[0], sel[1]
        else:
            return 'Align: Cannot determine nodes to align'

    targetMatrix = pmc.xform( target, q=True, ws=1, matrix=True )
    nodeMatrix = pmc.xform( node, q=True, ws=1, matrix=True )

    nodeScale = node.s.get()

    if translate and orient:
        pmc.xform(node, ws=1, matrix=targetMatrix)
    elif translate:
        # set row4 x y z to row4 of targetMatrix
        nodeMatrix[12:-1] = targetMatrix[ 12:-1]
        pmc.xform(node, ws=1, matrix=nodeMatrix)
    elif orient:
        # set rows 1-3 to rows 1-3 of nodeMatrix
        targetMatrix[12:-1] = nodeMatrix[12:-1]
        pmc.xform(node, ws=1, matrix=targetMatrix)

    if not scale:
        node.s.set(nodeScale)

    if parent:
        if not node.getParent() == target:
            node.setParent(target)

def duplicateJoints(joints, zeroRotations=1, name=''):
    '''

    :param joints: list of joints to duplicate
    :param zeroRotations: all rotation values will be applied to joint orient values and rot values will be zeroed
    :return: list of newly created joints
    '''
    if joints:
        newJoints = []
        sel = pmc.selected()
        for i in range(len(joints)):
            joint = joints[i]
            num = str(i+1).zfill(2)
            pmc.select(None)
            j = cmds.joint()
            if 'XX'in name:
                j = pmc.rename(j, name.replace('XX', num))
            if newJoints:
                j.setParent(newJoints[-1])
            j.jointOrientX.set(0)
            j.jointOrientY.set(0)
            j.jointOrientZ.set(0)
            align(j, joint)
            j.jointOrient.set(j.r.get())
            j.rx.set(0)
            j.ry.set(0)
            j.rz.set(0)
            newJoints.append(j)
        pmc.select(sel)
        return newJoints
    else:
        return 'duplicateJoints: Please supply some joints to duplicate'

def getShape(transform=None):
    '''
    returns the first shape of the specified transform

    '''
    shape = pmc.listRelatives(transform, children=1, shapes=1)[0]
    return shape

def blendAttrs(input1, input2, name, blendAttr):
    '''
    sets up blending of two float attrs using a blendTwoAttrNode
    If a blendAttr is supplied, this is connected to the attributesBlender attribute
    '''
    blend = pmc.createNode('blendTwoAttr', name=name)
    input1.connect(blend.input[1])
    input2.connect(blend.input[2])
    if blendAttr:
        blendAttr.connect(blend.attributesBlender)
    return blend

def blend(input1, input2, name, blendAttr=None):
    '''
    sets up blending of input1 and input2
    If a blendAttr is supplied, this is connected to the blender value
    '''
    val = 0.0
    connect=False

    blend = pmc.createNode('blendColors', name=name)

    if type(input1) == pmc.general.Attribute:
        val = input1.get()
        connect=True
    else:
        val = input1
        connect=False

    if type(val) == pmc.datatypes.Vector or type(val) == type((0, 0, 0)):
        if connect:
            input1.connect(blend.color1)
        else:
            blend.color1.set(input1)
    else:
        if connect:
            input1.connect(blend.color1R)
        else:
            blend.color1R.set(input1)

    if type(input2) == pmc.general.Attribute:
        val = input2.get()
        connect=True
    else:
        val = input2
        connect=False

    if type(val) == pmc.datatypes.Vector or type(val) == type((0, 0, 0)):
        if connect:
            input2.connect(blend.color2)
        else:
            blend.color2.set(input2)
    else:
        if connect:
            input2.connect(blend.color2R)
        else:
            blend.color2R.set(input2)

    if blendAttr:
        blendAttr.connect(blend.blender)

    return blend

def pairBlend(inTranslate1, inRotate1, inTranslate2, inRotate2, name, blendAttr=None):
    '''
    Sets up blending using a pairBlend node
    :param inTranslate1: node or attribute to connect to intTranslate1
    :param inRotate1: node or attribute to connect to intRotate1
    :param inTranslate2: node or attribute to connect to intTranslate2
    :param inRotate2: node or attribute to connect to intRotate2
    :param name: name of new pairBlend node
    :param blendAttr: attribute to connect to pairBlend weight
    :return: new pairBlend node
    '''
    val = 0.0

    def _checkInput(input):
        if type(input) == pmc.general.Attribute:
            return input
        elif pmc.nodetypes.Transform in type(input).__mro__:
            return input.t
        else:
            return False

    pb = pmc.createNode('pairBlend', name=name)
    val = _checkInput(inTranslate1)
    if val:
        val.connect(pb.inTranslate1)
    else:
        pb.inTranslate1.set(inTranslate1)

    val = _checkInput(inTranslate2)
    if val:
        val.connect(pb.inTranslate2)
    else:
        pb.inTranslate2.set(inTranslate2)

    val = _checkInput(inRotate1)
    if val:
        val.connect(pb.inRotate1)
    else:
        pb.inRotate1.set(inRotate1)

    val = _checkInput(inRotate2)
    if val:
        val.connect(pb.inRotate2)
    else:
        pb.inRotate2.set(inRotate2)

    if blendAttr:
        blendAttr.connect(pb.weight)

    return pb



def ikFkBlend(ik, fk, res, blendAttr=None):
    '''
    :param ik: ik input joint
    :param fk: fk input joint
    :param res: result joint
    :param blendAttr: optional - connects blending to this attribute if provided
    :return: newly created pairblend node
    '''
    pb = pmc.createNode('pairBlend', name='%s_ikFkBlend_utl' % res)
    ik.t.connect(pb.inTranslate1)
    ik.r.connect(pb.inRotate1)
    fk.t.connect(pb.inTranslate2)
    fk.r.connect(pb.inRotate2)
    pb.outTranslate.connect(res.t)
    pb.outRotate.connect(res.r)
    pb.rotInterpolation.set(1)
    res.rotateOrder.connect(pb.rotateOrder)

    if blendAttr:
        pmc.connectAttr(blendAttr, '%s.weight' % pb)

def getDistance( start, end ):
    '''
    Calculates distance between two Transforms using magnitude
    '''

    def mag(numbers):
        num = 0
        for eachNumber in numbers:
            num += math.pow(eachNumber, 2)

        mag = math.sqrt(num)
        return mag

    startPos, endPos = getStartAndEnd(start, end)

    if not startPos or not endPos:
        return 'getDistance: Cannot determine start and end points'

    calc = []
    calc.append(startPos[0] - endPos[0])
    calc.append(startPos[1] - endPos[1])
    calc.append(startPos[2] - endPos[2])

    return mag(calc)

def getStartAndEnd(start=None, end=None):
    '''
    Takes either two pynodes, two vectors or two selected nodes and returns their positions
    '''
    startPos, endPos = None, None
    if not start or not end:
        if len(pmc.selected()) == 2:
            startPos = pmc.xform(pmc.selected()[0], translation=True, query=True, ws=True)
            endPos = pmc.xform(pmc.selected()[1], translation=True, query=True, ws=True)
    else:
        if pmc.nodetypes.Transform in type(start).__mro__:
            startPos = pmc.xform(start, translation=True, query=True, ws=True)
        else:
            startPos = start

        if pmc.nodetypes.Transform in type(end).__mro__:
            endPos = pmc.xform(end, translation=True, query=True, ws=True)
        else:
            endPos = end

    if not startPos or not endPos:
        return (None, None)
    else:
        return startPos, endPos

def decomposeWorldMatrix(node):
    d = pmc.createNode('decomposeMatrix', name='%s_worldMtxToSrt_utl' % node.name())
    node.worldMatrix[0].connect(d.inputMatrix)
    return d

def decomposeMatrix(matrix, name):
    d = pmc.createNode('decomposeMatrix', name=name)
    matrix.connect(d.inputMatrix)
    return d

def connectAttrs(source, dest, attrList):
    '''
    :param source: node to connect attributes from
    :param dest: node to connect attributes to
    :param attrList: list of attributes to connect e.g. ['t', 'r', 's']
    :return:
    '''
    for attr in attrList:
        pmc.connectAttr('%s.%s' % (source, attr), '%s.%s' % (dest, attr))

def connectAttrToMany(sourceAttr, attrList):
    '''
    connects sourceAttr to each attr in attrList
    '''
    for attr in attrList:
        sourceAttr.connect(attr)

def decomposeLocalMatrix(node, depth=0):
    d = pmc.createNode('decomposeMatrix', name='%s_localMtxToSrt_utl' % node.name())
    if depth:
        m = pmc.createNode('multMatrix', name='%s_localMtx_utl' % node.name())
        node.matrix.connect(m.matrixIn[0])
        for i in range(depth):
            parent = node.getParent()
            parent.matrix.connect(m.matrixIn[str(i+1)])
        m.matrixSum.connect(d.inputMatrix)
    else:
        node.matrix.connect(d.inputMatrix)
    return d



def isDecomposed(node):
    # check to see if a decompose matrix is already connected to node
    conns = pmc.listConnections(node.worldMatrix[0], s=0, d=1, t='decomposeMatrix')
    d = None
    if conns:
        d = conns[0]
    else:
        d = decomposeWorldMatrix(node)
    return d


def connectDecomposedMatrix(source, dest):
    '''
    :param source: decomposeMatrix node to connect from
    :param dest: node to connect to
    :return:
    '''
    source.outputTranslate.connect(dest.t)
    source.outputRotate.connect(dest.r)
    source.outputScale.connect(dest.s)

def getAimMatrix(start=None, end=None, axis='x', upAxis='y', worldUpAxis='y', upNode=None):
    '''
    Given two nodes or two points, returns a matrix positioned at start, aiming at end along axis
    Similar to orient joint.
    '''
    startPos, endPos = getStartAndEnd(start, end)
    if not startPos or not endPos:
        return 'getAimMatrix: Unable to determine start and end positions'

    axisDict={'x':(1,0,0), 'y':(0,1,0), 'z':(0,0,1), '-x':(-1,0,0), '-y':(0,-1,0), '-z':(0,0,-1)}
    orderDict={'x':0, 'y':1, 'z':2, '-x':0, '-y':1, '-z':2}

    # Aim vector
    startVec = pmc.datatypes.Vector(startPos[0], startPos[1], startPos[2])
    endVec = pmc.datatypes.Vector(endPos[0], endPos[1], endPos[2])
    upVec = pmc.datatypes.Vector(axisDict[worldUpAxis])
    if upNode:
        upVec = pmc.datatypes.Vector(getAxisVector(node=upNode, axis=worldUpAxis))
    aimVec = None
    if '-' in axis:
        aimVec = startVec - endVec
    else:
        aimVec = endVec - startVec
    aimVec.normalize()
    normalVec = upVec.cross(aimVec)
    tangentVec = aimVec.cross(normalVec)

    matrixList = ['', '', '', startPos]
    matrixList[orderDict[axis]] = aimVec
    matrixList[orderDict[upAxis]] = tangentVec
    matrixList[matrixList.index('')] = normalVec
    outMatrix = pmc.datatypes.Matrix(matrixList)

    return outMatrix

def matrixFromVectors(row1, row2, row3, row4, name):
    '''
    Takes 3 tri vectors and constructs a fourbyfour matrix from them
    '''
    m = pmc.createNode('fourByFourMatrix', name=name)
    inputs = [m.in00, m.in01, m.in02, m.in10, m.in11, m.in12, m.in20, m.in21, m.in22, m.in30, m.in31, m.in32]
    outputs = row1.children() + row2.children() + row3.children() + row4.children()
    for i in range(12):
        outputs[i].connect(inputs[i])
    return m

def matrixToSrt(matrix):
    '''
    converts a pymel matrix into translate, rotate, scale values
    :param matrix: pymel.datatypes.Matrix to decompose
    :return: [translate, rotate, scale]
    '''
    t = matrix.translate
    #r = matrix.rotate
    r = (0, 0, 0)
    #s = matrix.scale
    s = (1,1,1)

    return {'translate':t, 'rotate':r, 'scale':s}

def getAxisVector(node=None, axis='x'):
    '''
    returns the world space vector representing the direction of node's axis
    '''
    matrix = pmc.xform(node, q=1, m=1, ws=1)
    axisDict = {'x':matrix[0:3], 'y':matrix[4:7], 'z':matrix[8:11], '-x':matrix[0:3], '-y':matrix[4:7], '-z':matrix[8:11]}
    outVec = axisDict[axis]
    if '-' in axis:
        outVec = [outVec[0]*-1, outVec[1]*-1, outVec[2]*-1]
    return outVec

def multiply(input1, input2, name, operation=1):
    '''
    creates a multiplyDivide node with the given inputs
    returns the newly created node

    if inputs are attributes, a connection to the attribute is made
    if inputs are values, initial md values are set accordingly

    '''
    md = pmc.createNode('multiplyDivide', name=name)
    md.operation.set(operation)

    val = 0.0
    connect=False

    if type(input1) == pmc.general.Attribute:
        val = input1.get()
        connect=True
    else:
        val = input1
        connect=False

    if type(val) == pmc.datatypes.Vector or type(val) == tuple:
        if connect:
            input1.connect(md.input1)
        else:
            md.input1.set(input1)
    else:
        if connect:
            input1.connect(md.input1X)
        else:
            md.input1X.set(input1)

    if type(input2) == pmc.general.Attribute:
        val = input2.get()
        connect=True
    else:
        val = input2
        connect=False

    if type(val) == pmc.datatypes.Vector or type(val) == tuple:
        if connect:
            input2.connect(md.input2)
        else:
            md.input2.set(input2)
    else:
        if connect:
            input2.connect(md.input2X)
        else:
            md.input2X.set(input2)

    return md

def multiplyDouble(input1, input2, name):
    md = pmc.createNode('multDoubleLinear', name=name)
    if type(input1) == pmc.general.Attribute:
        val = input1.get()
        connect=True
    else:
        val = input1
        connect=False

    if connect:
        input1.connect(md.input1)
    else:
        md.input1.set(input1)

    if type(input2) == pmc.general.Attribute:
        val = input2.get()
        connect=True
    else:
        val = input2
        connect=False

    if connect:
        input2.connect(md.input2)
    else:
        md.input2.set(input2)
    return md


def divide(input1, input2, name):
    md = multiply(input1, input2, name, operation=2)
    return md

def safeDivide(input1, input2, name):
    '''
    adds a condition node which switches the operation to multiply if the divisor == 0
    '''
    md = multiply(input1, input2, name)
    cond = pmc.createNode('condition', name='safeCond_%s' % name)
    input2.connect(cond.firstTerm)
    cond.colorIfTrueR.set(1)
    cond.colorIfFalseR.set(2)
    cond.outColorR.connect(md.operation)
    return md

def power(input1, input2, name):
    md = multiply(input1, input2, name, operation=3)
    return md

def add(inputs, name, operation=1):
    '''
    creates a plusMinusAverage node with the given inputs
    returns the newly created node

    if inputs are attributes, a connection to the attribute is made
    if inputs are values, initial md values are set accordingly
    '''
    pma = pmc.createNode('plusMinusAverage', name=name)
    pma.operation.set(operation)

    val = 0.0
    connect=False

    for i in range(len(inputs)):
        if type(inputs[i]) == pmc.general.Attribute:
            val = inputs[i].get()
            connect=True
        else:
            val = inputs[i]
            connect=False

        if type(val) == pmc.datatypes.Vector or type(val) == type((0, 0, 0)):
            if connect:
                inputs[i].connect('%s.input3D[%s]' % (pma, i))
            else:
                pmc.setAttr('%s.input3D[%s]' % (pma, i), inputs[i])
        else:
            if connect:
                inputs[i].connect('%s.input1D[%s]' % (pma, i))
            else:
                pmc.setAttr('%s.input1D[%s]' % (pma, i), inputs[i])

    return pma

def minus(inputs, name):
    pma = add(inputs, name, operation=2)
    return pma

def average(inputs, name):
    pma = add(inputs, name, operation=3)
    return pma

def distanceBetweenNodes(node1, node2, name):
    dist = pmc.createNode('distanceBetween', name=name)
    node1.worldMatrix[0].connect(dist.inMatrix1)
    node2.worldMatrix[0].connect(dist.inMatrix2)
    return dist

def distanceBetweenPoints(p1, p2, name):
    dist = pmc.createNode('distanceBetween', name=name)
    p1.connect(dist.point1)
    p2.connect(dist.point2)
    return dist

def convert(input, factor, name):
    '''
    creates a unit conversion node with input connected to input
    '''
    uc = pmc.createNode('unitConversion', name=name)
    input.connect(uc.input)
    uc.conversionFactor.set(factor)
    return uc

def reverse(input, name):
    '''
    creates a reverse node with input connected to input
    '''
    rev = pmc.createNode('reverse', name=name)
    input.connect(rev.inputX)
    return rev

#######################################################################################################################
### Vector product operations

def normalizeVector(input, name):
    '''
    :param input: vector to normalize
    :param name: name of new node
    :return: new node
    '''
    vp = pmc.createNode('vectorProduct', name=name)
    input.connect(vp.input1)
    vp.normalizeOutput.set(1)
    vp.operation.set(0)
    return vp

def cross(input1, input2, name, normalize=1):
    '''
    creates a vector product node to calculate the cross product between input1 and input2.
    Normalizes output by default
    '''
    vp = pmc.createNode('vectorProduct', name=name)
    input1.connect(vp.input1)
    input2.connect(vp.input2)
    vp.normalizeOutput.set(normalize)
    vp.operation.set(2)
    return vp

def dot(input1, input2, name, normalize=1):
    '''
    creates a vector product node to calculate the dot product between input1 and input2.
    Normalizes output by default
    '''
    vp = pmc.createNode('vectorProduct', name=name)
    input1.connect(vp.input1)
    input2.connect(vp.input2)
    vp.normalizeOutput.set(normalize)
    return vp

def pointMatrixMult(point, matrix, name, normalize=0):
    '''
    returns point transformed into the space of matrix
    '''
    connect = False
    vp = pmc.createNode('vectorProduct', name=name)
    if type(point) == pmc.general.Attribute:
        connect = True
    if connect:
        point.connect(vp.input1)
    else:
        vp.input1.set(point)
    matrix.connect(vp.matrix)
    vp.normalizeOutput.set(normalize)
    vp.operation.set(4)
    return vp

def matrixAxisToVector(obj, name, axis='x', normalize=1):
    '''
    Creates a vectorproduct node which outputs the representation of obj's matrix along 'axis'
    normalizes output by default
    '''
    vp = pmc.createNode('vectorProduct', name=name)
    if type(obj) == pmc.general.Attribute:
        obj.connect(vp.matrix)
    else:
        obj.worldMatrix.connect(vp.matrix)
    vp.normalizeOutput.set(normalize)
    vp.operation.set(3)
    pmc.setAttr('%s.input1%s' % (vp.name(), axis[-1].upper()), 1)
    return vp

def blendMatrices(mtx1, mtx2, weight1, weight2, name):
    '''
    Creates a wtAddMatrix node to blend between 2 matrices
    '''
    node = pmc.createNode('wtAddMatrix', name=name)
    mtx1.connect(node.wtMatrix[0].matrixIn)
    mtx2.connect(node.wtMatrix[1].matrixIn)
    node.wtMatrix[0].weightIn.set(weight1)
    node.wtMatrix[1].weightIn.set(weight2)

    return node

###
#######################################################################################################################

def isolateTwist(matrix1, matrix2, name, axis='x'):
    '''
    extracts the rotation along axis by multiplying matrix1 by matrix2
    and connecting only the relevant axes to a quatToEuler node.
    :param matrix1: matrix from which to extract the twist
    :param matrix2: worldInverseMatrix of comparison node
    :param name: prefix for newly created nodes
    :param axis: axis to extract
    :return: [multMatrix, decomposeMatrix, eulerToQuat]
    '''
    twistMtx = multiplyMatrices([matrix1, matrix2], name='%s_twistMtx_utl' % name)
    twistToSrt = decomposeMatrix(twistMtx.matrixSum, name='%s_twistMtxToSrt_utl' % name)
    twistQuat = pmc.createNode('quatToEuler', name='%s_twistSrtToQuat_utl' % name)
    rotOutAttr = pmc.Attribute('%s.outputQuat%s' % (twistToSrt.name(), axis[-1].upper()))
    rotInAttr = pmc.Attribute('%s.inputQuat%s' % (twistQuat.name(), axis[-1].upper()))
    rotOutAttr.connect(rotInAttr)
    twistToSrt.outputQuatW.connect(twistQuat.inputQuatW)
    rotOrderDict = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}
    twistQuat.inputRotateOrder.set(rotOrderDict[axis])

    return [twistMtx, twistToSrt, twistQuat]


def createAlignedNode(node=None, nodeType='group', name=''):
    '''
    creates a node of the specified type and aligns it to the provided node
    '''
    if node==None:
        if len(pmc.selected())==1:
            node=pmc.selected()[0]
        else:
            return 'Please select or specify a node to align to'

    alignedNode = None

    if nodeType =='group':
        alignedNode = pmc.group(empty=1)
    elif nodeType == 'locator':
        alignedNode = pmc.spaceLocator()
    elif nodeType == 'joint':
        alignedNode = pmc.joint()
        alignedNode.setParent(None)

    if alignedNode:
        align(alignedNode, node)

    if name:
        alignedNode.rename(name)

    return alignedNode

def pointsAlongVector( start='', end='', divisions=2, bias=0 ):
    '''
    returns a list of points that lie on a line between start and end
    'divisions' specifies the number of points to return.
    divisions = 2 (default) will return the start and end points with one intermediate point: [ p1(start), p2, p3(end) ]

    start and end can be supplied as lists, tuples or nodes. If they are not supplied, and 2 scene nodes are selected
    will attempt to use their world space positions as start and end

    creates a vector by subtracting end from start
    stores length of vector
    normalizes vector
    multiplies normalized vector by length / divisions

    bias specifies weight of point distribution towards start or end. Positive values favour end, negative favour start
    '''
    startPos, endPos = getStartAndEnd(start, end)

    if not startPos or not endPos:
        return 'pointsAlongVector: Cannot determine start and end points'

    startVec = om2.MVector(startPos[0], startPos[1], startPos[2])
    endVec = om2.MVector(endPos[0], endPos[1], endPos[2])
    newVec = endVec - startVec
    segLength = (newVec.length() / divisions) / newVec.length()
    #newVec.normalize()
    points = []
    points.append(tuple(startPos))

    segLengths = [segLength * each for each in range(divisions)]
    if bias:
        for i in range(divisions):
            #paramVals[i] - pow(paramVals[i], 1 + bias)
            segLengths[i] += segLengths[i] - pow(segLengths[i], 1 + bias)

    for p in range( 1, divisions ):
        point = (newVec * segLengths[p]) + startVec
        points.append((point.x, point.y, point.z))

    points.append(tuple(endPos))

    return points

def multiplyMatrices(matrixList, name):
    '''
    creates a multMatrix node to multiply matrices in matrixList
    :param matrixList: list of matrices to multiply
    :param name: name of new multMatrix node
    :return: new multMatrix node
    '''
    mm = pmc.createNode('multMatrix', name=name)
    for i in range(len(matrixList)):
        attr = pmc.Attribute('%s.matrixIn[%s]' % (mm.name(), str(i)))
        matrixList[i].connect(attr)
    return mm

def inverseMatrix(matrix, name):
    '''
    Creates an inverseMatrix node. connects matrix to it. Returns the new node
    '''
    inv = pmc.createNode('inverseMatrix', name=name)
    matrix.connect(inv.inputMatrix)
    return inv

def choose(inputList, name, choiceAttr=None):
    '''
    creates a choice node with inputs for each in inputList
    :param inputList: list of inputs
    :param choiceAttr: if supplied, the choice node's selector attribute will be connected to this attribute
    :param name: name of new choice node
    :return: new choice node
    '''
    choice = pmc.createNode('choice', name=name)
    for i in range(len(inputList)):
        inputList[i].connect('%s.input[%s]' % (choice.name(), i))
    if choiceAttr:
        choiceAttr.connect(choice.selector)
    return choice

def defineCollision(obj1, obj2, settingsNode, name, axis='y'):
    '''
    Compares the transforms of obj1 and obj2. If obj1 goes beyond obj2 on specified axis a blended transform is computed
    for output1 and output2. Otherwise output1=obj1.transform and output2=obj2.transform.
    :param obj1: first comparison object
    :param obj2: second comparison object
    :param settingsNode: node on which to expose outputs and config attributes
    :param name: prefix for newly created nodes
    :param axis: primary axis for comparision
    :return:
    '''
    pmc.addAttr(settingsNode, ln='push_amount', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
    pmc.addAttr(settingsNode, ln='push_midpoint', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
    pmc.addAttr(settingsNode, ln='secondary_push_mult', at='double', k=1, h=0, minValue=-1, maxValue=1.0)
    pmc.addAttr(settingsNode, ln='start_push', at='double', k=0)
    pmc.addAttr(settingsNode, ln='end_push', at='double', k=0)
    pmc.addAttr(settingsNode, ln='secondary_push', at='double', k=0)

    mtxMult = multiplyMatrices([obj1.worldMatrix[0], obj2.worldInverseMatrix[0]], name='%s_localMtx_utl' % name)
    d = decomposeMatrix(mtxMult.matrixSum, name='%s_localMtxToSrt_utl' % name)
    cond = pmc.createNode('condition', name='%s_isColliding_utl' % name)
    attr = pmc.Attribute('%s.outputTranslate%s' % (d.name(), axis.upper()))
    attr.connect(cond.firstTerm)
    cond.operation.set(2)

    midpointBlend = multiply(settingsNode.push_midpoint, cond.outColorR, name='%s_midpointBlend_utl' % name)
    startPushBlend = multiply(midpointBlend.outputX, attr, name='%s_startPushBlend_utl' % name)
    startPushAmount = multiply(startPushBlend.outputX, settingsNode.push_amount, '%s_startPushAmount_utl' % name)
    startPushAmount.outputX.connect(settingsNode.start_push)
    endPushPma = minus([startPushBlend.outputX, attr], name='%s_endPush_utl' % name)
    endPushBlend = multiply(endPushPma.output1D, cond.outColorR, name='%s_endPushBlend_utl' % name)
    endPushAmount = multiply(endPushBlend.outputX, settingsNode.push_amount, '%s_endPushAmount_utl' % name)
    endPushAmount.outputX.connect(settingsNode.end_push)

    secondaryPush = multiply(attr, cond.outColorR, name='%s_secondaryPush_utl' % name)
    secondaryPushBlend = multiply(secondaryPush.outputX, settingsNode.secondary_push_mult, name='%s_secondaryPushBlend_utl' % name)
    secondaryPushBlend.outputX.connect(settingsNode.secondary_push)


def attrCtrl(lock=True, keyable=False, channelBox=False, nodeList=[], attrList=[]):
    '''
    Takes a list of nodes and sets locks/unlocks shows/hides attributes in attrList
    '''
    if nodeList:
        for node in nodeList:
            if attrList:
                for a in attrList:
                    if node.hasAttr(a):
                        pmc.setAttr('%s.%s' % (node, a), lock=lock, keyable=keyable, channelBox=channelBox)
            else:
                return 'attrCtrl: No nodes supplied for attribute control'
    else:
        return 'attrCtrl: No nodes supplied for attribute control'

def copyWorldSpace(node):
    '''
    :param node: node to copy from
    :return: matrix representing nodes current transform in world space
    '''
    return pmc.xform(node, q=1, m=1, ws=1)

def pasteWorldSpace(node, mtx):
    pmc.xform(node, m=mtx, ws=1)


def getDominantAxis(start, end):
    '''
    returns the axis with the largest absolute translate value
    :param start: node to treat as parent
    :param end: node to treat as child
    :return: 'x', 'y', or 'z'
    '''
    tempStart = createAlignedNode(start, 'group', 'tempStart')
    tempEnd = createAlignedNode(end, 'group', 'tempEnd')
    tempEnd.setParent(tempStart)
    axisDict = {0: 'x', 1: 'y', 2: 'z'}
    endTranslateList = [abs(attr.get()) for attr in [tempEnd.tx, tempEnd.ty, tempEnd.tz]]
    axis = axisDict[endTranslateList.index(max(endTranslateList))]
    pmc.delete(tempStart)
    return axis

def selectSkinnedJoints(node=None):
    '''
    selects all joints bound to the specified node
    '''
    if node==None:
        if len(pmc.selected())==1:
            node=pmc.selected()[0]
        else:
            return 'Please select or specify a skinCluster node'

    influences = pmc.skinCluster(node, q=1, influence=1)
    pmc.select(influences)

def vectorBetweenNodes(start, end, name):
    '''
    Creates nodes to calculate the vector from start to end nodes. Checks if either node's matrix has been decomposed,
    if not creates decomposeMatrix nodes. Subtracts, start position from end position using a plusminusaverage node.
    Returns plusminusAverage node.
    '''
    dStart = isDecomposed(start)
    dEnd = isDecomposed(end)
    pma = minus([dEnd.outputTranslate, dStart.outputTranslate], name=name)
    return pma

def forceAbsolute(attr, name):
    '''
    Creates two multiply divide nodes set to power. The first squares the value of attr (pow2), the second get the
    square root of the first (pow0.5)
    returns the second node
    '''
    mdSquare = power(attr, 2.0, name='%s_square_utl' % name)
    mdSquareRoot = power(mdSquare.outputX, 0.5, name='%s_squareRoot_utl' % name)
    return [mdSquare, mdSquareRoot]