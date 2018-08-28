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
            driver.out_translate[index].connect(j.t, f=1)
            driver.out_rotate[index].connect(j.r, f=1)
            driver.out_scale.connect(j.s, f=1)

##----------------------------------------------------------------------------------------------------------------------
#
# IMPORTING AND CONNECTING FACE RIG
#
##----------------------------------------------------------------------------------------------------------------------
def mergeRig():
    filePath = cmds.file(q=1, l=1)[0].rsplit('/', 1)[0]
    fileName = None
    try:
        fileName = [f for f in os.listdir(filePath) if os.path.isfile(os.path.join(filePath, f)) and 'RIG_face.ma' in f][0]
    except:
        raise RuntimeError('No RIG_face.ma file found in this directory: %s' % filePath)
    cmds.file(fileName, i=1, namespace='face')

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
def mergeJoints():
    filePath = cmds.file(q=1, l=1)[0].rsplit('/', 1)[0]
    fileName = None
    try:
        fileName = [f for f in os.listdir(filePath) if os.path.isfile(os.path.join(filePath, f)) and 'SKIN_face.ma' in f][0]
    except:
        raise RuntimeError('No SKIN_face.ma file found in this directory: %s' % filePath)
    cmds.file(fileName, i=1, namespace='face')
    for node in pmc.ls('face:*'):
        try:
            node.setParent('joints_grp')
        except:
            pass
    pmc.namespace(removeNamespace='face', mergeNamespaceWithRoot=1)

def loadWeights():
    filePath = cmds.file(q=1, l=1)[0].rsplit('/', 1)[0]
    files = [f for f in os.listdir(filePath) if os.path.isfile(os.path.join(filePath, f)) and 'faceWeights.json' in f]

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
        #pmc.setAttr('%s.maintainMaxInfluences' % skin, 1)

        # Get list of joints from weights file
        weights = None
        with open(os.path.join(filePath, f), 'r') as jsonFile:
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