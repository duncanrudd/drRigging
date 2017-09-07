import pymel.core as pmc
import json


def messageConnect(fromNode=None, toNode=None, fromName=None, toName=None, category=None):
    '''
    Creates a message attribute on fromNode and toNode with the names fromName and toName respectively
    Connects the two new attributes
    '''
    # validation
    if not fromNode or not toNode and (len(pmc.selected()) == 2):
        fromNode = pmc.selected()[0]
        toNode = pmc.selected()[1]

    if not fromNode or not toNode:
        return 'Argument Error, messageConnect requires fromNode and toNode as either arguments or 2 selected nodes'

    if not fromName or not toName:
        return 'Argument Error, messageConnect requires fromName and toName arguments for newly created attrs'

    # Add attributes
    if pmc.attributeQuery(fromName, node=fromNode, exists=1):
        print '%s.%s: Attribute exists' % (fromNode, fromName)
    else:
        pmc.addAttr(fromNode, ln=fromName, at='message', category=category)
        print '%s.%s: Attribute created' % (fromNode, fromName)

    if pmc.attributeQuery(toName, node=toNode, exists=1):
        print '%s.%s: Attribute exists' % (toNode, toName)
    else:
        pmc.addAttr(toNode, ln=toName, at='message', category=category)
        print '%s.%s: Attribute created' % (toNode, toName)

    # Connect attributes
    pmc.connectAttr('%s.%s' % (fromNode, fromName), '%s.%s' % (toNode, toName), f=1)
    print '%s.%s connected to %s.%s' % (fromNode, fromName, toNode, toName)


def parentConnect(parent=None, child=None):
    '''
    Specific method for connecting parent / child relationships in a metarig
    '''
    # Validation
    if not parent or not child and (len(pmc.selected()) == 2):
        parent = pmc.selected()[0]
        child = pmc.selected()[1]

    if not parent or not child:
        return 'Argument Error, Please supply a parent and child node'

    if parent in getAllMetaChildren(child):
        return '%s is a meta descendent of %s and cannot also be its parent' % (parent, child)

    # Disconnect from old parent's children array
    oldParent = pmc.listConnections('%s.metaParent' % child)

    if type(oldParent) == type([]):
        metaChildren = getMetaChildren(oldParent[0])
        childIndex = metaChildren.index(child)
        pmc.disconnectAttr('%s.message' % child, '%s.metaChildren[%s]' % (oldParent[0], childIndex))
    # Connect to new parent's children array
    metaChildren = getMetaChildren(parent)
    pmc.connectAttr('%s.message' % child, '%s.metaChildren[%s]' % (parent, len(metaChildren)))

    # Connect new parent
    messageConnect(fromNode=parent, toNode=child, fromName='message', toName='metaParent')


def getMetaChildren(node=None):
    '''
    returns a list of all metaChildren of Node

    '''
    if not node and len(pmc.selected()) == 1:
        node = pmc.selected()[0]
    if not node:
        return 'Please supply a node whose children you wish to list'

    metaChildren = pmc.listConnections('%s.metaChildren' % node)
    if not metaChildren:
        metaChildren = []

    return metaChildren


def getAllMetaChildren(node=None):
    '''
    returns a list of all metaDescendents of Node
    '''
    if not node and len(pmc.selected()) == 1:
        node = pmc.selected()[0]
    if not node:
        return 'Please supply a node whose descendents you wish to list'

    metaChildren = []

    def __getAllMetaChildrenRecurse__(node):
        mc = getMetaChildren(node)
        if mc:
            for n in mc:
                print n
                pass
                __getAllMetaChildrenRecurse__(n)
        metaChildren.append(node)

    mc = getMetaChildren(node)
    for n in mc:
        __getAllMetaChildrenRecurse__(n)

    return metaChildren


def addDictionaryAttr(node=None, dictName=None):
    '''
    Creates a custom attribute which stores a json encoded dictionary on the specified node
    '''
    # Validation
    if not node and (len(pmc.selected()) == 1):
        node = pmc.ls(sl=1)[0]

    if not node:
        return 'Argument Error, addDictionaryAttr requires node as either argument or a selected node'

    if not dictName:
        return 'Argument Error, addDictionaryAttr requires dictName for newly created dictionary'

        # Add attributes
    if pmc.attributeQuery(dictName, node=node, exists=1):
        return 'Argument Error, %s.%s: Dictionary Attribute exists' % (node, dictName)
    else:
        pmc.addAttr(node, ln=dictName, dt="string")
        print '%s.%s: Dictionary Attribute created' % (node, dictName)

    # Set initial dictionary value
    pmc.setAttr('%s.%s' % (node, dictName), json.dumps({}), type="string")


def readDict(dict=None):
    '''
    reads a string containing a dictionary and parses it using json
    '''
    return json.loads(dict)