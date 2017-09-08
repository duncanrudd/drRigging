import maya.cmds as cmds
import drRigging.utils.coreUtils as coreUtils
import pymel.core as pmc


def orientCtrl(ctrl=None, axis=None):
    '''
    rotates the cvs of a ctrl to point along axis

    '''
    # orient ik_anim cvs properly
    shape = coreUtils.getShape(ctrl)
    pmc.select('%s.cv[ * ]' % shape)

    if axis == 'y':
        pmc.rotate(90, rotateX=True)
    elif axis == 'x':
        pmc.rotate(90, rotateY=True)
    elif axis == 'z':
        pmc.rotate(90, rotateZ=True)
    elif axis == '-y':
        pmc.rotate(-90, rotateX=True)
    elif axis == '-x':
        pmc.rotate(-90, rotateY=True)
    elif axis == '-z':
        pmc.rotate(-90, rotateZ=True)

    pmc.select(None)


def circleCtrl(radius=20.0, name='', axis='z'):
    '''
    creates a circular nurbs curve

    '''
    ctrl = pmc.circle(name=name, r=radius, ch=0, o=1)[0]

    if axis != 'z':
        orientCtrl(ctrl=ctrl, axis=axis)
    return ctrl


def circleBumpCtrl(radius=20.0, name='', axis='z'):
    '''
    creates a circular nurbs curve with a bump to indicate orientation

    '''
    ctrl = pmc.circle(name=name, r=radius, ch=0, o=1, s=24)[0]

    shape = coreUtils.getShape(ctrl)
    shape.rename(ctrl.name() + 'Shape')
    pmc.select('%s.cv[ 1 ]' % shape)
    pmc.move(radius * .5, moveY=1, r=1)
    if '-' in axis:
        pmc.select('%s.cv[*]' % shape)
        pmc.rotate(180, rotateZ=True)

    if axis != 'z':
        orientCtrl(ctrl=ctrl, axis=axis)
    return ctrl


def boxCtrl(size=20.0, name=''):
    '''
    Creates a box shaped nurbs curve

    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(neg, pos, neg), (neg, neg, neg), (neg, neg, pos), (neg, pos, pos),
              (neg, pos, neg), (pos, pos, neg), (pos, neg, neg), (neg, neg, neg),
              (neg, neg, pos), (pos, neg, pos), (pos, neg, neg), (pos, pos, neg),
              (pos, pos, pos), (neg, pos, pos), (pos, pos, pos), (pos, neg, pos)]

    knots = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]

    ctrl = pmc.curve(degree=1, p=points, k=knots, name=name)

    return ctrl


def crossCtrl(size=20.0, name='', axis='z'):
    '''
    Creates a locator shaped nurbs curve

    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(0, 0, neg), (0, 0, pos), (0, 0, 0), (0, pos, 0),
              (0, neg, 0), (0, 0, 0), (pos, 0, 0), (neg, 0, 0)]

    knots = [1, 2, 3, 4, 5, 6, 7, 8]

    ctrl = pmc.curve(degree=1, p=points, k=knots, name=name)

    return ctrl


def squareCtrl(size=20.0, name='', axis='y'):
    '''
    creates a square nurbs curve

    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(neg, neg, 0), (neg, pos, 0), (pos, pos, 0), (pos, neg, 0), (neg, neg, 0)]

    knots = [1, 2, 3, 4, 5]

    ctrl = pmc.curve(degree=1, p=points, k=knots, name=name)
    if axis != 'z':
        orientCtrl(ctrl=ctrl, axis=axis)
    return ctrl

def squareChamferCtrl(size=20.0, name='', axis='y'):
    '''
    creates a square nurbs curve

    '''
    points = [(-1, 0, -3), (-1, 0, -4), (1, 0, -4), (1, 0, -3), (3, 0, -1), (4, 0, -1), (4, 0, 1),(3, 0, 1),
              (1, 0, 3), (1, 0, 4), (-1, 0, 4), (-1, 0, 3), (-3, 0, 1),(-4, 0, 1), (-4, 0, -1), (-3, 0, -1), (-1, 0, -3)]

    knots = [i for i in range(len(points))]

    ctrl = pmc.curve(degree=1, p=points, k=knots, name=name)
    shape = coreUtils.getShape(ctrl)
    pmc.select('%s.cv[ * ]' % shape)
    pmc.rotate(45, rotateY=True)
    pmc.scale(size*.25, size*.25, size*.25)
    if axis != 'z':
        orientCtrl(ctrl=ctrl, axis=axis)
    return ctrl

def ballCtrl(radius=20.0, name=''):
    ctrl = pmc.circle(name=name, r=radius, ch=0, o=1, s=8, nr=(0,1,0))[0]
    dup1 = pmc.circle(name=name, r=radius, ch=0, o=1, s=8, nr=(1,0,0))[0]
    dup2 = pmc.circle(name=name, r=radius, ch=0, o=1, s=8, nr=(0,0,1))[0]
    shape = pmc.listRelatives(dup1, shapes=True)[0]
    pmc.parent(shape, ctrl, s=1, r=1)
    shape = pmc.listRelatives(dup2, shapes=True)[0]
    pmc.parent(shape, ctrl, s=1, r=1)
    pmc.delete([dup1, dup2])

    return ctrl


def pinCtrl(radius=20.0, name='', axis='z'):
    '''
    creates a pin control

    '''
    axisDict = {'x': (radius, 0, 0), 'y': (0, radius, 0), 'z': (0, 0, radius), '-x': (radius * -1, 0, 0),
                '-y': (0, radius * -1, 0), '-z': (0, 0, radius * -1)}
    line = pmc.curve(d=1, p=[(0, 0, 0), axisDict[axis]], k=[0, 1], name=name)

    circle = pmc.circle(name=(name + '_circle'), r=radius * .2, ch=0, o=1, s=24)
    if axis != 'z':
        orientCtrl(ctrl=circle, axis=axis)
    shape = pmc.listRelatives(circle, shapes=True)[0]

    cmds.move(axisDict[axis][0], axisDict[axis][1], axisDict[axis][2], "%s.cv[:]" % shape, r=1)

    pmc.parent(shape, line, shape=1, r=1)
    shapes = pmc.listRelatives(line, shapes=True)
    shapes[1].rename(shapes[0].nodeName().replace('Shape', 'CircleShape'))
    pmc.delete(circle)

    return line









