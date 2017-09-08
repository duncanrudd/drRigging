import drRigging.utils.coreUtils as coreUtils
import drRigging.objects.controls as controls
reload(coreUtils)
reload(controls)

import pymel.core as pmc

def hasSpaceSwitch(node):
    '''
    checks to see whether a choice node is connected to node's translate or rotate or both
    :param node: node to check
    :return: [t, r, p] where t is the choice node connected to node's translate or 0 if not found,
    r is the choice node connected to node's rotate or 0 if not found,
    p is the choice node connected to node's translate and rotate if it is the same node connected to both
    '''
    t, r, p = 0, 0, 0
    translateMtx = pmc.listConnections(node.t, d=0, type='decomposeMatrix')
    rotateMtx = pmc.listConnections(node.r, d=0, type='decomposeMatrix')
    if translateMtx:
        translateConns = pmc.listConnections(translateMtx[0].inputMatrix, d=0, type='choice')
        if translateConns:
            t = translateConns[0]
    if rotateMtx:
        rotateConns = pmc.listConnections(rotateMtx[0].inputMatrix, d=0, type='choice')
        if rotateConns:
            if rotateConns[0] == t:
                p = t
                t = 0
            else:
                r = rotateConns[0]
    return {'translate': t, 'rotate': r, 'parent': p}

def addSpaceSwitch(node, name, spaces, type='parent', ctrlNode=None, targetNodes=[]):

    enumString = ''
    startIndex = 0
    asset = getComponentFromName(node)
    choice = hasSpaceSwitch(node)[type]
    attr = None
    add = 0
    if choice:
        add = 1
        ctrlNode = pmc.listConnections(choice.selector, d=0)[0]
        attr = pmc.listConnections(choice.selector, d=0, p=1)[0]
        attrName = attr.name().split('.')[-1]
        enumString = pmc.attributeQuery(attrName, node=ctrlNode, listEnum=1)[0] + ':'
        startIndex = len(enumString.split(':'))-1
        pmc.deleteAttr(attr)
    else:
        choice = pmc.createNode('choice', name='%s_%s_%sSpaceChoice_utl' % (asset, name, type))
        if not ctrlNode:
            ctrlNode = node


    for space in spaces:
        enumString += (space + ':')
    pmc.addAttr(ctrlNode, ln='%sSpace' % type, at='enum', enumName=enumString, k=1, h=0)
    attr = pmc.Attribute('%s.%sSpace' % (ctrlNode.name(), type))
    attr.connect(choice.selector)

    targets = []
    for i in range(len(spaces)):
        space = spaces[i]
        g = coreUtils.addChild(getComponentGroup(node, group='input'), 'group', name='%s_%s_%s_%s_in_srt' % (asset, name, space, type))
        #coreUtils.align(g, node)
        #g.worldMatrix[0].connect(pmc.Attribute('%s.input[%s]' % (choice.name(), i + startIndex)))
        targets.append(g)
        if len(targetNodes)==len(spaces):
            targetNode = targetNodes[i]
            if getComponentFromName(node) == getComponentFromName(targetNode):
                if targetNode in pmc.listRelatives(getComponentGroup(targetNode, 'rig'), c=1, ad=1) or targetNode in pmc.listRelatives(getComponentGroup(targetNode, 'input'), c=1, ad=1):
                    g.setParent(targetNode)
                elif targetNode in pmc.listRelatives(getComponentGroup(targetNode, 'interface'), c=1, ad=1):
                    g.setParent(getComponentGroup(targetNode, 'rig'))
                    d = coreUtils.isDecomposed(targetNode)
                    coreUtils.connectDecomposedMatrix(d, g)
            else:
                connectIO(g, targetNode, space)
        offset = coreUtils.addChild(g, 'group', name='%s_%s_%s_%s_offset_srt' % (asset, name, space, type))
        coreUtils.align(offset, node)
        offset.worldMatrix[0].connect(pmc.Attribute('%s.input[%s]' % (choice.name(), i + startIndex)))

    if not add:
        d = coreUtils.decomposeMatrix(choice.output, name='%s_%s_%sSpaceMtxToSrt_utl' % (asset, name, type))
        if type == 'parent':
            coreUtils.connectDecomposedMatrix(d, node)
        elif type == 'translate':
            d.outputTranslate.connect(node.t)
        elif type == 'rotate':
            d.outputRotate.connect(node.r)

    return targets

def removeSpaceSwitch(node, settingsNode, type='parent'):
    pass


def exposeOutput(node, name=''):
    '''
    creates a transform in the output group of node's asset which matches node's worldspace transforms
    :param node: node whose transform to expose
    :return: newly created output group
    '''
    if not name:
        name = node.name().split('_')[2]
    d = coreUtils.isDecomposed(node)
    g = coreUtils.addChild(getComponentGroup(node), 'group', '%s_%s_out_srt' % (getComponentFromName(node), name))
    d.outputTranslate.connect(g.t)
    d.outputRotate.connect(g.r)
    d.outputScale.connect(g.s)

    return g

def exposeInput(node, source, name):
    '''

    '''
    input = getInput(node, name)
    if not input:
        input = coreUtils.addChild(getComponentGroup(node, group='input'), 'group',
                                   '%s_%s_in_srt' % (getComponentFromName(node), name))
    outSource = getOutput(source)
    if not outSource:
        outSource = exposeOutput(source, name)

    outSource.t.connect(input.t)
    outSource.r.connect(input.r)
    outSource.s.connect(input.s)

    return input

def exposeDeformation(node, name):
    '''
    creates a joint in the deform group of node's asset witch matches node's worldspace transforms
    :param node: node whose transform to expose
    :return: newly created deform joint
    '''
    d = coreUtils.isDecomposed(node)
    j = coreUtils.addChild(getComponentGroup(node, group='deform'), 'joint', '%s_dfm' % name)
    j.jointOrient.set((0,0,0))
    d.outputTranslate.connect(j.t)
    d.outputRotate.connect(j.r)
    d.outputScale.connect(j.s)
    return j


def getComponentFromName(node):
    '''
    returns the name of the asset that the supplied node belongs to by tokenising the name using '_'
    :param node: node whose asset to return
    :return: string in the form 'asset_side' eg 'arm_L'
    '''
    tokens = node.name().split('_')
    return '%s_%s' % (tokens[0], tokens[1])


def getComponentGroup(node, group='output'):
    '''
    returns the output group of the asset to which node belongs
    :param node: node whose asset to return
    :return: pynode representing the output group of the current node's asset
    '''
    asset = getComponentFromName(node)
    try:
        return pmc.PyNode('%s_%s' % (asset, group))
    except:
        return "unable to find %s group for %s's asset" % (group, node.name())


def getOutput(node):
    '''
    searches for an output connected to node.
    :param node: node to search for outputs on (can be interface node or output node
    :return: the output srt node if found or None if not found
    '''
    outputGrp = getComponentGroup(node, 'output')
    for output in pmc.listRelatives(outputGrp, c=1):
        if node == output:
            return output
        mtxConns = pmc.listConnections(output.t, d=0)
        if mtxConns:
            conns = pmc.listConnections(mtxConns[0].inputMatrix, d=0)
            if node in conns:
                return output
    return None

def getInput(node, source):
    '''
    searches for an srt in the inputs group of node with connections to source.
    :param node: node whose component input group to search within
    :param source: srt output node to look for connections to
    :return: the input srt node if found or None if not found
    '''
    inputGrp = getComponentGroup(node, 'input')
    for input in pmc.listRelatives(inputGrp, c=1):
        conns = pmc.listConnections(input, d=0)
        if source in conns:
            return input
    return None

def connectIO(dest, source, connectionName):
    '''
    checks for an output from source. If it finds one, creates a corresponding input on dest's component input group
    If an input already exists it will be used instead. Also creates an offset group under the input which
    is the node to decompose and connect things to within the component
    :param dest: node to drive from the new input (usually a ctrl's buffer_srt
    :param source: node to connect the input to
    :param connectionName: name for the new input connection
    :return: offset group under new input node
    '''
    if dest.getParent() == getComponentGroup(dest, 'input'):
        # check whether source is in worldspace. If so apply a direct connection
        offset = coreUtils.addChild(getComponentGroup(dest, 'input'), 'group', '%s_%s_in_srt' % (getComponentFromName(dest), connectionName))
        if source.worldMatrix.get() == source.matrix.get():
            source.t.connect(offset.t)
            source.r.connect(offset.r)
            source.s.connect(offset.s)
        else:
            d = coreUtils.isDecomposed(source)
            coreUtils.connectDecomposedMatrix(d, offset)
        dest.setParent(offset)
    elif getComponentFromName(dest) == getComponentFromName(source):
        if dest.getParent() == getComponentGroup(dest, 'interface') or dest.getParent() == getComponentGroup(dest, 'rig'):

            print 'creating rig input'
            # This means source and dest are in the same component and a target can be added to the rig group
            offset = coreUtils.createAlignedNode(dest, 'group', '%s_%s_%sOffset_srt' % (getComponentFromName(dest), connectionName, dest.name().split('_')[2]))
            rels = pmc.listRelatives(getComponentGroup(dest, 'rig'), ad=1, c=1)
            if source in rels:
                offset.setParent(source)
            else:
                input = coreUtils.addChild(getComponentGroup(dest, 'rig'), 'group', '%s_%s_%s_srt' % (getComponentFromName(dest), connectionName, dest.name().split('_')[2]))
                if source.worldMatrix.get() == source.matrix.get():
                    source.t.connect(input.t)
                    source.r.connect(input.r)
                    source.s.connect(input.s)
                else:
                    d = coreUtils.isDecomposed(source)
                    coreUtils.connectDecomposedMatrix(d, input)
                offset.setParent(input)
            d = coreUtils.isDecomposed(offset)
            coreUtils.connectDecomposedMatrix(d, dest)
    else:
        output = getOutput(source)
        if not output:
            output = exposeOutput(source)
        input = getInput(dest, output)
        print 'creating new input'
        input = exposeInput(dest, output, connectionName)
        conns = pmc.listConnections(dest.t, d=0, type='decomposeMatrix')
        offset = None
        if conns:
            offset = pmc.listConnections(conns[0].inputMatrix, d=0)[0]
            if offset.getParent() != input:
                offset.setParent(input)
                offset.rename(offset.getParent().name().replace('_srt', '_%sOffset_srt' % dest.name().split('_')[2]))
        else:
            offset = coreUtils.createAlignedNode(dest, 'group',
                                                 '%s_%s_in_%sOffset_srt' % (getComponentFromName(dest),
                                                                      connectionName,
                                                                      dest.name().split('_')[2]))
            offset.setParent(input)

            # Check whether dest is in worldspace. If so, use a direct connection. Otherwise, multiply by parent mtx
            if dest.worldMatrix.get() != dest.matrix.get():
                m = coreUtils.multiplyMatrices([offset.worldMatrix[0], dest.getParent().worldInverseMatrix],
                                               '%s_%s_localMtx_utl' % (getComponentFromName(dest), connectionName))
                d = coreUtils.decomposeMatrix(m.matrixSum, '%s_%s_localMtxToSrt_utl' % (getComponentFromName(dest), connectionName))
                coreUtils.connectDecomposedMatrix(d, dest)
            else:
                d = coreUtils.isDecomposed(offset)
                coreUtils.connectDecomposedMatrix(d, dest)
        return offset

def addCtrl(anchor, target, name, ctrlSize=1):
    '''
    adds an ik control to anchor's component
    The control will be aligned to target and attached to anchor
    :param name: name for new control e.g. 'shldr_R'
    :return: new control
    '''
    if anchor in pmc.listRelatives(getComponentGroup(anchor, 'interface'), ad=1):
        print 'anchor is is interface'
        c = controls.boxCtrl(size=ctrlSize, name='%s_%s_ctrl' % (getComponentFromName(anchor), name))
        b = coreUtils.addParent(c, 'group', '%s_%s_buffer_srt' % (getComponentFromName(anchor), name))
        coreUtils.align(b, target, scale=1)
        b.setParent(anchor)
        return c
    elif anchor in pmc.listRelatives(getComponentGroup(anchor, 'input'), ad=1) or anchor in pmc.listRelatives(getComponentGroup(anchor, 'rig'), ad=1):
        print 'anchor is in rig or input'
        c = controls.boxCtrl(size=ctrlSize, name='%s_%s_ctrl' % (getComponentFromName(anchor), name))
        b = coreUtils.addParent(c, 'group', '%s_%s_buffer_srt' % (getComponentFromName(anchor), name))
        coreUtils.align(b, target, scale=1)
        offset = coreUtils.addChild(anchor, 'group', '%s_%s_offset_srt' % (getComponentFromName(anchor), name))
        coreUtils.align(offset, b, scale=1)
        b.setParent(getComponentGroup(anchor, 'interface'))
        d = coreUtils.isDecomposed(offset)
        coreUtils.connectDecomposedMatrix(d, b)
        return c
    else:
        print 'anchor is not in rig, input or interface'
        return 'You can only attach controls to nodes in the input, interface or rig groups of a component'