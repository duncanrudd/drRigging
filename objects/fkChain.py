import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.objects.controls as controls
reload(coreUtils)
reload(controls)

def fkChainBetweenPoints(start, end, name, bias, numCtrls=3, ctrlInterval=3):
    '''
    Creates an fk chain aligned to start finishing at end.
    :param start: beginning of the chain
    :param end: end of the chain
    :param name: prefix for newly created nodes
    :param numCtrls: number of fk ctrls to distribute along the chain
    :param bias: weight of control distribution towards start or end. Positive values favour end, negative favour start
    :param ctrlInterval: number of intermediate transforms between controls
    :return: list of ctrls
    '''
    ctrls = []
    points = coreUtils.pointsAlongVector(start, end, (numCtrls*ctrlInterval), bias)
    ctrlSize = coreUtils.getDistance(start, end)

    # get axis for control alignment
    axis = coreUtils.getDominantAxis(start, end)

    # Base
    baseCtrl = controls.circleBumpCtrl(radius=ctrlSize*.2, name='%s_base_ctrl' % name, axis=axis)
    baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % name)
    coreUtils.align(baseBuffer, start)
    ctrls.append(baseCtrl)

    #FK
    for i in range(len(points)):
        num = str(i+1).zfill(2)
        ctrlNum = str((i / ctrlInterval) +1 ).zfill(2)
        c = None
        if i % ctrlInterval == 0 and i != len(points)-1:
            c = controls.circleBumpCtrl(name='%s_%s_ctrl' % (name, ctrlNum),
                                        axis=axis, radius=ctrlSize*.1)
        else:
            c = coreUtils.pmc.group(empty=1, name='%s_%s_srt' % (name, num))
            ctrls[-1].t.connect(c.t)
            ctrls[-1].r.connect(c.r)
            ctrls[-1].s.connect(c.s)
        b = coreUtils.addParent(c, 'group', name='%s_%s_buffer_srt' % (name, num))
        if i == 0:
            coreUtils.align(b, start)
            b.setParent(baseCtrl)
        else:
            b.t.set(points[i])
            coreUtils.align(b, start, translate=0)
            b.setParent(ctrls[-1])
        ctrls.append(c)
