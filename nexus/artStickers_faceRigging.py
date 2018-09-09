import maya.OpenMaya as OpenMaya
import maya.OpenMayaAnim as OpenMayaAnim
import json
import maya.cmds as cmds
import pymel.core as pmc
import os

##----------------------------------------------------------------------------------------------------------------------
#
# CONNECTING JOINTS TO RIG
#
##----------------------------------------------------------------------------------------------------------------------
def getOutputs():
    outputDict = {}
    outputs = [node for node in pmc.ls(type='transform') if pmc.hasAttr(node, 'outputType')]
    for output in outputs:
        outputDict[output.outputType.get()] = output
    return outputDict

def getDrivenJoints(rigType):
    drivenJoints = [j for j in pmc.ls(type='joint') if pmc.hasAttr(j,'rigType') and j.rigType.get() == rigType]
    return drivenJoints

def connectJoints():
    outputDict = getOutputs()
    for rigType in outputDict.keys():
        driver = outputDict[rigType]
        joints = getDrivenJoints(rigType)
        for j in joints:
            index = j.jointIndex.get()
            m = pmc.createNode('multMatrix', name=j.name().replace('Out_Jnt', 'jntMtx_utl'))
            driver.outMatrix[index].connect(m.matrixIn[0])
            j.parentInverseMatrix[0].connect(m.matrixIn[1])
            d = pmc.createNode('decomposeMatrix', name=j.name().replace('Out_Jnt', 'jntMtx2Srt_utl'))
            m.matrixSum.connect(d.inputMatrix)
            d.outputTranslate.connect(j.t)
            d.outputRotate.connect(j.r)
            d.outputScale.connect(j.s)
            j.jointOrient.set((0,0,0))

def connectFaceElemsInComboRig(comboRigPath):
    cmds.file(new=1, force=1)
    cmds.file(comboRigPath, o=1)
    connectJoints()

##----------------------------------------------------------------------------------------------------------------------
#
# IMPORTING AND CONNECTING FACE RIG
#
##----------------------------------------------------------------------------------------------------------------------
def mergeRig(faceRigPath):
    cmds.file(faceRigPath, i=1, namespace='face')
    pmc.namespace(removeNamespace='face', mergeNamespaceWithRoot=1)

def getInputs():
    inputs = [node for node in pmc.ls(type='transform') if pmc.hasAttr(node, 'rigInputs')]
    return inputs

def getParents():
    parents = [node for node in pmc.ls(type='transform') if pmc.hasAttr(node, 'rigParent')]
    return parents

def connectRigs():
    inputs = getInputs()
    for input in inputs:
        for element in input.rigInputs.elements():
            attr = pmc.Attribute('%s.%s' % (input.name(), element))
            splitString = attr.get().split('>>')
            sourceAttr = pmc.Attribute(splitString[0])
            destAttr = pmc.Attribute('%s.%s' % (input.name(), splitString[1]))
            sourceAttr.connect(destAttr, f=1)

    parents = getParents()
    for p in parents:
        p.setParent(pmc.PyNode(p.rigParent.get()))
    pmc.sets('all_ctls_SEL', add='face_ctls_SEL')

def installFaceRigIntoMainRig(faceRigPath,mainRigPath):
    if mainRigPath is not None:
        cmds.file(new=1, force=1)
        cmds.file(mainRigPath, o=1)
    mergeRig(faceRigPath)
    connectRigs()

##----------------------------------------------------------------------------------------------------------------------
#
# SAVING WEIGHTS
#
##----------------------------------------------------------------------------------------------------------------------
def getSkin(node):
    skin = [x for x in pmc.listHistory(node) if pmc.nodeType(x) == "skinCluster" ]
    if skin:
        return skin[0].name()
    else:
        return None

def saveWeights(mesh):
    filePath = cmds.file(q=1, l=1)[0].rsplit('/', 1)[0]
    fileName = filePath + '/%s_faceWeights.json' % mesh.name()

    skin = getSkin(mesh)

    if not skin:
        return 'no skin cluster found on %s' % node.name()

    # get the MFnskin for skin
    selList = OpenMaya.MSelectionList()
    selList.add(skin)
    skinNode = OpenMaya.MObject()
    selList.getDependNode(0, skinNode)
    skinFn = OpenMayaAnim.MFnSkinCluster(skinNode)

    # get the MDagPath for all influence
    infDags = OpenMaya.MDagPathArray()
    skinFn.influenceObjects(infDags)

    joints = [str(infDags[i].fullPathName().split('|')[-1]) for i in range(infDags.length())]
    print joints

    # get the MPlug for the weightList and weights attributes
    wlPlug = skinFn.findPlug('weightList')
    wPlug = skinFn.findPlug('weights')
    wlAttr = wlPlug.attribute()
    wAttr = wPlug.attribute()
    wInfIds = OpenMaya.MIntArray()

    weights = {}
    for vId in xrange(wlPlug.numElements()):
        vWeights = {}
        # tell the weights attribute which vertex id it represents
        wPlug.selectAncestorLogicalIndex(vId, wlAttr)

        # get the indice of all non-zero weights for this vert
        wPlug.getExistingArrayAttributeIndices(wInfIds)

        # create a copy of the current wPlug
        infPlug = OpenMaya.MPlug(wPlug)
        for infId in wInfIds:
            # tell the infPlug it represents the current influence id
            infPlug.selectAncestorLogicalIndex(infId, wAttr)

            # add this influence and its weight to this verts weights
            try:
                vWeights[str(infDags[infId].fullPathName().split('|')[-1])] = infPlug.asDouble()
            except KeyError:
                # assumes a removed influence
                pass
        weights[vId] = vWeights

    weights['joints'] = joints

    with open(fileName, "w") as jsonFile:
            json.dump(weights, jsonFile, indent=2)

    return '%s saved succesfully' % fileName

for node in pmc.selected():
    print saveWeights(node)


##----------------------------------------------------------------------------------------------------------------------
#
# IMPORTING FACE JOINTS AND LOADING WEIGHTS
#
##----------------------------------------------------------------------------------------------------------------------
def mergeJoints(faceSkinPath):

    cmds.file(faceSkinPath, i=1, namespace='face')
    for j in pmc.ls('face:*', type='joint'):
        try:
            j.setParent(pmc.ls('Neck_??_Out_Jnt')[-1])
        except:
            j.setParent(pmc.PyNode('Body_Out_Jnt'))
        pmc.disconnectAttr(j.getParent().s, j.inverseScale)

    pmc.namespace(removeNamespace='face', mergeNamespaceWithRoot=1)

def loadWeights(weightsPath):

    files = [f for f in os.listdir(weightsPath) if os.path.isfile(os.path.join(weightsPath, f)) and 'faceWeights.json' in f]

    for f in files:
        mesh = None
        try:
            mesh = pmc.PyNode(f.split('_faceWeights.json')[0])
        except:
            continue
        skin = getSkin(mesh)

        if not skin:
            print 'no skin cluster found on %s' % node.name()
            continue
        #pmc.setAttr('%s.maxInfluences' % skin, 4)
        pmc.setAttr('%s.maintainMaxInfluences' % skin, 0)
        pmc.setAttr('%s.weightDistribution' % skin, 1)

        # Get list of joints from weights file
        weights = None
        with open(os.path.join(weightsPath, f), 'r') as jsonFile:
          weights = json.load(jsonFile)

        # Make sure joints from weights file are added to skinCluster
        joints = None
        try:
            joints = [pmc.PyNode(j) for j in weights['joints'] if not j == 'null_jnt']
            print joints
        except:
            raise RuntimeError('Some joints specified in the weights file not found in scene: %s' % weights['joints'])
        try:
            pmc.skinCluster(skin, ai=joints, lw=1, wt=0, e=1)
        except:
            print 'joints are already in %s: %s' % (skin, joints)

        # Apply weights
        for v in weights.keys():
            if v == 'joints':
                continue
            keys = [key for key in weights[v].keys() if key != 'null_jnt']
            values = [weights[v][key] for key in keys]
            pmc.skinPercent(skin, '%s.vtx[%s]' % (mesh, v), transformValue=list(zip(keys, values)))

def installFaceSkinIntoMainSkin(faceSkinPath,mainSkinPath,weightsPath):
    if mainSkinPath is not None:
        cmds.file(new=1, force=1)
        cmds.file(mainSkinPath, o=1)
    mergeJoints(faceSkinPath)
    loadWeights(weightsPath)

##----------------------------------------------------------------------------------------------------------------------
#
# UTILITIES
#
##----------------------------------------------------------------------------------------------------------------------

def unMirrorMtx(mtx, name):
    m = pmc.createNode('composeMatrix', name='%s_unMirrorMtx_utl' % name)
    m.inputScaleX.set(-1)
    mult = pmc.createNode('multMatrix', name='%s_mtx_utl' % name)
    m.outputMatrix.connect(mult.matrixIn[0])
    mtx.connect(mult.matrixIn[1])
    return mult

def exposeOutput(ctrl, outputNode, rigType, unMirror=0, createJoint=1):
    if pmc.hasAttr(outputNode, 'outMatrix'):
        index = outputNode.outMatrix.numElements()
        if unMirror:
            m = unMirrorMtx(ctrl.worldMatrix[0], name=ctrl.name().replace('ctrl', ''))
            m.matrixSum.connect(outputNode.outMatrix[index])
        else:
            ctrl.worldMatrix[0].connect(outputNode.outMatrix[index])
        if createJoint:
            pmc.select(None)
            j = pmc.joint(name=ctrl.name().replace('ctrl', 'Out_Jnt'))
            j.segmentScaleCompensate.set(0)

            pmc.addAttr(j, ln='jointIndex', at='short', k=0)
            pmc.addAttr(j, ln='rigType', dt='string')
            j.rigType.set(rigType)
            j.jointIndex.set(index)
            d = pmc.createNode('decomposeMatrix', name=ctrl.name().replace('ctrl', 'mtx2Srt_utl'))
            outputNode.outMatrix[index].connect(d.inputMatrix)
            d.outputTranslate.connect(j.t)
            d.outputRotate.connect(j.r)
            d.outputScale.connect(j.s)
            return j

def copyWeights(sourceMesh, destMesh):
    '''
    Select skinned mesh then non-skinned mesh and run: copyWeights(pmc.selected()[0], pmc.selected()[1])
    '''

    sourceSkin = [x for x in pmc.listHistory(sourceMesh) if pmc.nodeType(x) == "skinCluster" ][0]
    influences = pmc.skinCluster(sourceMesh, q=1, influence=1)
    pmc.select(influences)

    destSkin = pmc.skinCluster(influences, destMesh)
    pmc.copySkinWeights( ss=sourceSkin.name(), ds=destSkin.name(), noMirror=True )